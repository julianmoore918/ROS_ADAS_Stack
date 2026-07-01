#!/usr/bin/env python3
"""
ADAS UI — drive the CARLA + custom-bridge + ROS ADAS stack from one window.

Buttons:
  * Start / Stop CARLA           — CARLA 0.9.16 server on the chosen RPC port
  * Start / Stop Bridge          — carlaaccsim/carlaAccSimTown.py
                                   (publishes /Car_1/camera/front/compressed,
                                    /Car_1/vehicle/speed, subscribes /Car_1/cmd_vel)
  * Run start_acc.sh             — launches all four ADAS nodes via the script
  * Stop ADAS Stack              — kills start_acc.sh + any orphan ADAS nodes
  * ACC: ON/OFF                  — toggles perception_node + controller_node
  * LKAS: ON/OFF                 — toggles lane_detection_node + stanley_node

The right side of the window renders the live camera feed by subscribing
to /Car_1/camera/front/compressed (rclpy, runs in a background thread).

Run with system Python 3.10 (ROS-sourceable, has rclpy + PIL + cv2 + numpy).
"""
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, scrolledtext

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, Float64
    CAMERA_AVAILABLE = True
    _camera_err = None
except ImportError as e:
    CAMERA_AVAILABLE = False
    _camera_err = str(e)


# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------
ADAS_WK       = Path(__file__).resolve().parent
ADAS_INSTALL  = ADAS_WK / 'install' / 'setup.bash'
CARLA_DIR     = Path('/home/sirius/CARLA_0.9.16')

# Lane-detection model dropdown options: (display_name, model_ref).
# `model_ref` is passed to lane_detection_node via -p model_filename:=…
# A bare filename resolves against share/perception/models/; an absolute
# path is used as-is. Add new trained checkpoints below as they become
# available — no other UI code needs to change.
LANE_MODELS: list[tuple[str, str]] = [
    ('Current best (UFLD_best.pth)', 'UFLD_best.pth'),
    # After the 20260701 retrain finishes, add e.g.:
    # ('Retrained 20260701',
    #  '/home/sirius/workspace/01_CV_Models/'
    #  '01_Ultra_Fast_Lane_Detection_V2/logs/carla_res34/'
    #  '20260701_092451_lr_1e-02_b_16carla_finetune/model_best.pth'),
]

# Same pattern as LANE_MODELS but for the YOLO object detector consumed
# by perception_node. Add newly-trained YOLO checkpoints below.
OBJECT_MODELS: list[tuple[str, str]] = [
    ('Current best YOLO (best.pt)', 'best.pt'),
    # Example after a retrain:
    # ('Retrained YOLO 20260710',
    #  '/home/sirius/workspace/Trained_YOLO/runs/detect/train2/'
    #  'weights/best.pt'),
]
CARLA_SERVER  = './CarlaUE4.sh'
CARLA_INI     = Path('/home/sirius/CARLA_0.9.16/CarlaUE4/Config/DefaultEngine.ini')
CARLA_PYTHON  = Path('/home/sirius/CARLA_0.9.16/carla-env/bin/python3')
BRIDGE_DIR    = Path('/home/sirius/workspace/carlaaccsim')
BRIDGE_SCRIPT = BRIDGE_DIR / 'carlaAccSimTown.py'
START_ACC_SH  = ADAS_WK / 'start_acc.sh'
ROS_SETUP     = '/opt/ros/humble/setup.bash'

CAMERA_TOPIC       = '/Car_1/camera/front/compressed'
ACC_DEBUG_TOPIC    = '/ACC/perception/debug_image'
LKAS_DEBUG_TOPIC   = '/LKAS/perception/debug_image'
FUSED_DEBUG_TOPIC  = '/ADAS/perception/debug_image'
IPM_DEBUG_TOPIC    = '/ADAS/ipm/debug_image'
CMD_VEL_TOPIC      = '/Car_1/cmd_vel'
CMD_STEER_TOPIC    = '/Car_1/cmd_steer'
SPEED_TOPIC        = '/Car_1/vehicle/speed'

# Display-name → topic for the camera-source selector. The debug topics
# carry YOLO bounding boxes (ACC), UFLD ego-lane polylines (LKAS), and
# the combined view (ADAS), drawn server-side by the perception nodes.
# IPM_DEBUG_TOPIC has its own permanent BEV panel to the right of the
# camera (see _build_ui), so it's intentionally NOT in this dict.
CAMERA_SOURCES = {
    'Raw':              CAMERA_TOPIC,
    'ACC (YOLO)':       ACC_DEBUG_TOPIC,
    'LKAS (UFLD)':      LKAS_DEBUG_TOPIC,
    'ADAS (YOLO+UFLD)': FUSED_DEBUG_TOPIC,
}

# BEV display widget — native IPM image is 320×480 (see ipm_view_node).
# We render at 1.125× upscale to make it more legible without being
# huge. Aspect ratio (2:3) is preserved in the BEV-only render path.
BEV_W = 320
BEV_H = 480

# A node is "alive" if its heartbeat topic has published within this window.
NODE_ALIVE_WINDOW_S = 1.5

# Tkinter doesn't enjoy a flood of PhotoImage swaps. Bridge publishes at
# ~20 Hz; downsample to ~12 Hz for the widget.
CAMERA_UI_HZ  = 12
# Target render size for the camera widget (preserves the 16:9 bridge feed).
CAMERA_W      = 960
CAMERA_H      = 540

TOWNS = [
    'Town01', 'Town02', 'Town03', 'Town04', 'Town05',
    'Town10HD_Opt', 'Town01_Opt', 'Town02_Opt', 'Town03_Opt',
    'Town04_Opt', 'Town05_Opt', 'Town10HD',
]

WEATHER_PRESETS = [
    'ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon',
    'MidRainyNoon', 'HardRainNoon', 'SoftRainNoon',
    'ClearSunset', 'CloudySunset', 'WetSunset', 'HardRainSunset',
    'ClearNight', 'CloudyNight',
]

TRAFFIC_OPTIONS = ['0', '5', '10', '20', '30', '50']

# Junction policy — display name → bridge `--junction-policy` value. The
# bridge's CARLA-map junction monitor revokes LKAS steer authority inside
# any junction zone; the policy decides who owns steer instead. Mirrors
# the `--policy` set in 00_Lane_Assistant/02_UFLD_V2/UI.py.
JUNCTION_POLICIES = {
    'Pure pursuit': 'pp-takeover',
    'Hold straight': 'hold-straight',
}


# --------------------------------------------------------------------------
# Helpers — CARLA-side state via the carla-env Python
# --------------------------------------------------------------------------
def set_boot_map(ini_path: Path, town: str):
    """Rewrite the three *.Map entries in CARLA's DefaultEngine.ini so the
    next server boot lands in the requested town. (In-band load_world()
    segfaults on this install — the boot-map ini is the only reliable way.)
    Returns (n_lines_changed, new_value, observed_after_write)."""
    text = ini_path.read_text()
    new_value = f'/Game/Carla/Maps/{town}.{town}'
    pat = re.compile(r'(EditorStartupMap|GameDefaultMap|ServerDefaultMap)=.+')
    new_text, n = pat.subn(rf'\1={new_value}', text)
    ini_path.write_text(new_text)
    os.sync()
    after = ini_path.read_text()
    m = re.search(r'ServerDefaultMap=(.+)', after)
    observed = m.group(1).strip() if m else '<missing>'
    return n, new_value, observed


# Each of these snippets is fed to the carla-env Python via `-c`. They
# connect to the running CARLA, mutate state, and exit — no long-lived
# subprocess. Failures bubble up to the caller for logging.
_WEATHER_SNIPPET = """
import carla, sys
preset = sys.argv[1]
port = int(sys.argv[2])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
world.set_weather(getattr(carla.WeatherParameters, preset))
print(f'[weather] applied {preset}')
"""

_TRAFFIC_SPAWN_SNIPPET = """
import carla, random, sys, time
n = int(sys.argv[1])
port = int(sys.argv[2])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
random.shuffle(spawn_points)
vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*')
               if bp.has_attribute('number_of_wheels')
               and int(bp.get_attribute('number_of_wheels')) == 4]

# Use the bridge's default TM port (8000). The previous `port + 6000`
# math landed on 8000 only when CARLA was on port 2000 and silently
# created a *second* TM at 8002 / 8003 / etc. on any other CARLA port.
# Always-default keeps UI and bridge on the same shared TM.
tm = client.get_trafficmanager()
tm.set_global_distance_to_leading_vehicle(2.5)
tm.global_percentage_speed_difference(30.0)
# Critical: explicitly force TM async to match world. TM state persists
# across processes on the CARLA server, so a prior session that left
# this TM in sync mode would freeze every NPC we spawn here.
tm.set_synchronous_mode(False)
# Hybrid physics mode ALSO persists server-side across sessions. With
# it on, NPCs far from any hero are dormant (no physics, no motion) —
# looks identical to "autopilot disengaged". Force it off so every
# spawned NPC gets full physics regardless of distance to the ego.
tm.set_hybrid_physics_mode(False)
# Belt-and-braces: if any NPC ends up dormant despite the line above
# (CARLA can mark distant actors dormant under memory pressure), TM
# will revive it instead of leaving it stuck.
try:
    tm.set_respawn_dormant_vehicles(True)
except Exception:
    pass  # older CARLA builds may not have this method

# Atomic spawn + autopilot via batch commands. Without the .then()
# chaining, there was a window between try_spawn_actor returning and
# set_autopilot landing in which the vehicle existed but had no
# controller — it drifted on default zero-throttle / centred-steer
# and crashed into curbs / other actors. Exactly the bug we fixed
# in carlaaccsim/carlaAccSimTown.py:spawn_traffic; same fix needed
# here because UI.py spawns NPCs independently of the bridge.
SpawnActor   = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor  = carla.command.FutureActor

batch = []
for sp in spawn_points[:n]:
    bp = random.choice(vehicle_bps)
    if bp.has_attribute('color'):
        colors = bp.get_attribute('color').recommended_values
        if colors:
            bp.set_attribute('color', random.choice(colors))
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'npc')   # cleared by clear-snippet
    batch.append(
        SpawnActor(bp, sp)
        .then(SetAutopilot(FutureActor, True, tm.get_port()))
    )

# due_tick_cue=True so the server processes the SpawnActor + the
# chained SetAutopilot together within this call and we know the
# autopilot attachment has *landed* before the subprocess exits.
# With False the batch was queued and returned immediately — by the
# time the next async tick processed it, the snippet had already
# exited and its TM client connection had died, leaving freshly-
# spawned NPCs orphaned (registered with TM but never controlled).
# Matches the bridge's spawn_traffic which uses True and works.
spawned_ids = []
for resp in client.apply_batch_sync(batch, True):
    if not resp.error:
        spawned_ids.append(resp.actor_id)
print(f'[traffic] spawned {len(spawned_ids)}/{n} NPCs (atomic batch, async TM)',
      flush=True)

# Long-lived heartbeat loop. The previous 2 s sleep wasn't enough —
# NPCs were registered with TM via the batch's SetAutopilot, but the
# moment this subprocess exited and its TM client connection died,
# the NPCs either lost autopilot or were dropped by TM (CARLA 0.9.16
# tracks the registering client per-vehicle, and orphaned vehicles
# freeze even if other clients — like the bridge — are still
# connected to the same TM). Keeping the client alive forever fixes
# that, and re-asserting set_autopilot every few seconds also
# rescues any vehicle TM might have forgotten for any reason.
# The clear-snippet still finds these NPCs by role_name='npc' and
# destroys them; killing this subprocess (UI shutdown / restart)
# then releases TM ownership cleanly.
import signal
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
print('[traffic] heartbeat loop started — keep this process alive to '
      'maintain NPC autopilot. Will re-assert every 5 s.', flush=True)
while True:
    time.sleep(5.0)
    npcs = [a for a in world.get_actors().filter('vehicle.*')
            if a.attributes.get('role_name') == 'npc']
    for a in npcs:
        try:
            a.set_autopilot(True, tm.get_port())
        except Exception:
            pass
    print(f'[traffic] heartbeat: {len(npcs)} NPCs in autopilot',
          flush=True)
"""

_TRAFFIC_CLEAR_SNIPPET = """
import carla, sys
port = int(sys.argv[1])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
killed = 0
for a in world.get_actors().filter('vehicle.*'):
    if a.attributes.get('role_name') == 'npc':
        try:
            a.destroy(); killed += 1
        except Exception:
            pass
print(f'[traffic] cleared {killed} NPCs')
"""

# Print every spawn point in the currently-loaded map. Format mirrors
# the bridge's own --list-spawns flag so log output is consistent
# whether you ask via UI or CLI.
_LIST_SPAWNS_SNIPPET = """
import carla, sys
port = int(sys.argv[1])
client = carla.Client('localhost', port); client.set_timeout(10.0)
spawns = client.get_world().get_map().get_spawn_points()
for i, sp in enumerate(spawns):
    loc, rot = sp.location, sp.rotation
    print(f'  {i:3d}  x={loc.x:8.2f}  y={loc.y:8.2f}  '
          f'z={loc.z:6.2f}  yaw={rot.yaw:7.2f}')
print(f'[spawns] {len(spawns)} spawn points')
"""


# --------------------------------------------------------------------------
# ROS subscriber — camera sources + node heartbeats
# --------------------------------------------------------------------------
class TelemetryView(Node):
    """Subscribes to:
      * the three camera-source topics (raw + ACC debug + LKAS debug) so the
        UI can switch what it renders without tearing down subscriptions;
      * the controller / Stanley command topics so the UI can show which
        nodes are actually publishing (perception_node and lane_detection_node
        are covered by the debug-image subs).
    Camera frames are stashed as raw JPEG bytes; decode happens in the Tk
    render tick so we only pay the cost for the source being shown. Every
    received message also updates a last-seen timestamp so the UI can render
    ACC / LKAS alive indicators.
    """

    def __init__(self):
        super().__init__('adas_ui_telemetry')
        self.latest_jpegs = {t: None for t in CAMERA_SOURCES.values()}
        # IPM lives outside CAMERA_SOURCES (separate widget, not source-
        # selectable). Tracked independently so the BEV panel's render
        # path doesn't go through _active_camera_topic / latest_jpegs.
        self.latest_bev_jpeg: bytes | None = None
        self.last_seen: dict[str, float] = {}
        # Ego speed in m/s. None until the bridge publishes a first sample.
        self.speed_mps: float | None = None

        for topic in CAMERA_SOURCES.values():
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._on_image(t, msg), 10)
        self.create_subscription(
            CompressedImage, IPM_DEBUG_TOPIC, self._on_bev, 10)
        self.create_subscription(
            Twist, CMD_VEL_TOPIC,
            lambda _msg: self._touch(CMD_VEL_TOPIC), 10)
        self.create_subscription(
            Float32, CMD_STEER_TOPIC,
            lambda _msg: self._touch(CMD_STEER_TOPIC), 10)
        self.create_subscription(
            Float64, SPEED_TOPIC, self._on_speed, 10)

    def _on_speed(self, msg):
        self.speed_mps = float(msg.data)
        self._touch(SPEED_TOPIC)

    def _on_image(self, topic, msg):
        # Stash raw bytes; let the render tick decide whether to decode.
        self.latest_jpegs[topic] = bytes(msg.data)
        self._touch(topic)

    def _on_bev(self, msg):
        # Same idea as _on_image but for the always-on BEV panel.
        self.latest_bev_jpeg = bytes(msg.data)
        self._touch(IPM_DEBUG_TOPIC)

    def _touch(self, topic: str):
        self.last_seen[topic] = time.monotonic()

    def is_alive(self, topic: str) -> bool:
        ts = self.last_seen.get(topic)
        return ts is not None and (time.monotonic() - ts) < NODE_ALIVE_WINDOW_S


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
class ADASUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title('ADAS UI — CARLA + ACC + LKAS')

        # Long-lived child processes.
        self.carla_proc: subprocess.Popen | None = None
        self.bridge_proc: subprocess.Popen | None = None
        self.stack_proc: subprocess.Popen | None = None         # start_acc.sh
        self.foxglove_proc: subprocess.Popen | None = None      # foxglove_bridge
        self.acc_procs: list[subprocess.Popen] = []             # when toggled independently
        self.lkas_procs: list[subprocess.Popen] = []
        # NPC spawner runs the long-lived TRAFFIC_SPAWN_SNIPPET. The
        # snippet stays alive in a heartbeat loop to keep its TM client
        # connected — without that, CARLA 0.9.16 drops per-vehicle
        # autopilot ownership and the NPCs freeze. Tracked so Spawn
        # can replace the previous spawner, Clear can kill it before
        # destroying the actors (otherwise the heartbeat would just
        # re-attach autopilot on cars Clear is about to destroy), and
        # UI shutdown can clean it up so it doesn't survive past the UI.
        self.npc_spawn_proc: subprocess.Popen | None = None

        self.acc_on = False
        self.lkas_on = False

        # ROS subscriber for the camera widget + node-alive heartbeats.
        self.ros_node: TelemetryView | None = None
        self.ros_thread: threading.Thread | None = None

        # Frame throttling + active source.
        self._camera_source_var: tk.StringVar | None = None  # set in _build_ui
        self._render_after = None
        self._last_rendered_jpeg: bytes | None = None  # avoid re-decoding the same frame
        self._last_bgr: np.ndarray | None = None        # last decoded BGR frame (for video writer)
        # BEV (always-on /ADAS/ipm/debug_image) — same idempotency trick
        # so we don't re-decode a jpeg we already painted.
        self._last_rendered_bev_jpeg: bytes | None = None

        # Video recorder. None when idle; cv2.VideoWriter while recording.
        # Records whatever the camera widget is currently showing — switching
        # source mid-recording therefore changes what gets written, which is
        # the natural UX (you record what you see).
        self._video_writer: 'cv2.VideoWriter | None' = None
        self._video_path: Path | None = None
        self._video_size: tuple[int, int] | None = None  # (w, h) frozen at record start

        self._build_ui()
        self._start_camera_view()

    # --------------------------------------------------------------------
    # UI construction
    # --------------------------------------------------------------------
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.grid(row=0, column=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        # Column 0 = controls (fixed width), col 1 = camera + log (grows
        # with window width), col 2 = permanent BEV panel (fixed width).
        # weight=1 only on col 1 so the BEV preserves its native aspect
        # without stretching when the window is resized.
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(0, weight=1)

        # Left column: controls.
        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky='nw', padx=(0, 8))

        ttk.Label(left, text='RPC port').grid(row=0, column=0, sticky='w')
        self.port_var = tk.StringVar(value='2000')
        ttk.Entry(left, textvariable=self.port_var, width=8).grid(
            row=0, column=1, sticky='w', padx=4, pady=2)

        ttk.Label(left, text='Quality').grid(row=1, column=0, sticky='w')
        self.quality_var = tk.StringVar(value='Epic')
        ttk.Combobox(left, textvariable=self.quality_var, values=['Epic', 'Low'],
                     state='readonly', width=6).grid(
            row=1, column=1, sticky='w', padx=4, pady=2)

        ttk.Label(left, text='Town').grid(row=2, column=0, sticky='w')
        self.town_var = tk.StringVar(value='Town03')
        self.town_combo = ttk.Combobox(left, textvariable=self.town_var,
                                        values=TOWNS, state='readonly', width=16)
        self.town_combo.grid(row=2, column=1, sticky='w', padx=4, pady=2)
        self.town_combo.bind('<<ComboboxSelected>>', self._on_town_change)

        ttk.Label(left, text='Weather').grid(row=3, column=0, sticky='w')
        self.weather_var = tk.StringVar(value='ClearNoon')
        ttk.Combobox(left, textvariable=self.weather_var, values=WEATHER_PRESETS,
                     state='readonly', width=16).grid(
            row=3, column=1, sticky='w', padx=4, pady=2)
        ttk.Button(left, text='Apply Weather', command=self.apply_weather).grid(
            row=4, column=0, columnspan=2, sticky='ew', padx=4, pady=(0, 2))

        ttk.Label(left, text='Traffic (NPCs)').grid(row=5, column=0, sticky='w')
        self.traffic_var = tk.StringVar(value='0')
        ttk.Combobox(left, textvariable=self.traffic_var, values=TRAFFIC_OPTIONS,
                     state='readonly', width=8).grid(
            row=5, column=1, sticky='w', padx=4, pady=2)
        traffic_btns = ttk.Frame(left)
        traffic_btns.grid(row=6, column=0, columnspan=2, sticky='ew', padx=4, pady=(0, 2))
        traffic_btns.columnconfigure(0, weight=1)
        traffic_btns.columnconfigure(1, weight=1)
        ttk.Button(traffic_btns, text='Spawn Traffic', command=self.spawn_traffic).grid(
            row=0, column=0, sticky='ew')
        ttk.Button(traffic_btns, text='Clear Traffic', command=self.clear_traffic).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))

        # Process group.
        procs = ttk.LabelFrame(left, text='Processes', padding=6)
        procs.grid(row=7, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        procs.columnconfigure(0, weight=1)
        procs.columnconfigure(1, weight=1)
        ttk.Button(procs, text='Start CARLA', command=self.start_carla).grid(
            row=0, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop CARLA', command=self.stop_carla).grid(
            row=0, column=1, sticky='ew', pady=2, padx=(4, 0))
        ttk.Button(procs, text='Restart CARLA', command=self.restart_carla).grid(
            row=1, column=0, columnspan=2, sticky='ew', pady=2)
        ttk.Button(procs, text='Start Bridge', command=self.start_bridge).grid(
            row=2, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop Bridge', command=self.stop_bridge).grid(
            row=2, column=1, sticky='ew', pady=2, padx=(4, 0))
        # Spawn index — passed as --spawn-index to the bridge at Start
        # Bridge. Index into the current town's spawn_points list; the
        # ego appears there and the lead `--lead-gap-m` ahead. Takes
        # effect on the next Start Bridge. The "List" button dumps every
        # spawn point (index, x, y, z, yaw) into the log so the user
        # can pick one — CARLA must be running.
        ttk.Label(procs, text='Spawn index:').grid(
            row=3, column=0, sticky='w', pady=(0, 4))
        spawn_frame = ttk.Frame(procs)
        spawn_frame.grid(row=3, column=1, sticky='w', pady=(0, 4))
        self.spawn_index_var = tk.StringVar(value='0')
        ttk.Spinbox(spawn_frame, from_=0, to=999, width=6,
                    textvariable=self.spawn_index_var).pack(side='left')
        ttk.Button(spawn_frame, text='List',
                   command=self.list_spawns, width=6).pack(
            side='left', padx=(4, 0))
        # Junction policy — passed as --junction-policy to the bridge at
        # Start Bridge. Switching mid-run has no effect; restart the bridge.
        # See DEBUG.md §13.
        ttk.Label(procs, text='Junction policy:').grid(
            row=4, column=0, sticky='w', pady=(0, 4))
        self.junction_policy_var = tk.StringVar(value='Pure pursuit')
        ttk.Combobox(procs, textvariable=self.junction_policy_var,
                     values=list(JUNCTION_POLICIES.keys()),
                     state='readonly', width=14).grid(
            row=4, column=1, sticky='w', pady=(0, 4))
        # Rosbag recording toggle — passed as --record to the bridge at
        # Start Bridge. Off by default (bridge used to leave a GB/min
        # rosbag on disk every run whether the operator wanted it or
        # not). Ticking it means the next Start Bridge will record.
        self.rosbag_record_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(procs, text='Record rosbag',
                        variable=self.rosbag_record_var).grid(
            row=5, column=0, columnspan=2, sticky='w', pady=(0, 4))
        ttk.Button(procs, text='Run start_acc.sh', command=self.run_start_acc).grid(
            row=6, column=0, columnspan=2, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop ADAS Stack', command=self.stop_stack).grid(
            row=7, column=0, columnspan=2, sticky='ew', pady=2)
        # Foxglove bridge — independent visualisation tool. Opens
        # ws://localhost:8765 for Foxglove Studio (desktop or web).
        # Lives outside the ADAS/CARLA lifecycle so layouts can also be
        # used for rosbag playback after the stack is stopped.
        ttk.Button(procs, text='Start Foxglove',
                   command=self.start_foxglove).grid(
            row=8, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop Foxglove',
                   command=self.stop_foxglove).grid(
            row=8, column=1, sticky='ew', pady=2, padx=(4, 0))

        # Feature toggles. Each row has a button (user intent — ON/OFF) and a
        # small status dot reflecting whether the backing nodes are actually
        # publishing on their heartbeat topics. Green = publishing, grey = silent.
        feats = ttk.LabelFrame(left, text='Features', padding=6)
        feats.grid(row=8, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        feats.columnconfigure(0, weight=1)

        # ── Object-detection (YOLO) model selector, above ACC ────────
        # Populated from OBJECT_MODELS at the top of this file. Passed
        # to perception_node as -p model_filename:=<ref> at ACC: ON.
        self.object_model_var = tk.StringVar(value=OBJECT_MODELS[0][0])
        obj_row = ttk.Frame(feats)
        obj_row.grid(row=0, column=0, columnspan=2, sticky='ew',
                     pady=(0, 2))
        obj_row.columnconfigure(1, weight=1)
        ttk.Label(obj_row, text='Object model:').grid(
            row=0, column=0, sticky='w')
        ttk.Combobox(obj_row, textvariable=self.object_model_var,
                     values=[d for d, _ in OBJECT_MODELS],
                     state='readonly', width=32).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))

        self.acc_btn = ttk.Button(feats, text='ACC: OFF', command=self.toggle_acc)
        self.acc_btn.grid(row=1, column=0, sticky='ew', pady=2)
        self.acc_status_var = tk.StringVar(value='○ idle')
        self.acc_status_lbl = ttk.Label(feats, textvariable=self.acc_status_var,
                                         foreground='#888888', width=12)
        self.acc_status_lbl.grid(row=1, column=1, sticky='w', padx=(6, 0))

        # ── Lane-detection (UFLD) model selector, above LKAS ─────────
        # See LANE_MODELS at the top. Passed to lane_detection_node as
        # -p model_filename:=<ref> at LKAS: ON. No filesystem scan at
        # build time (would crash if _log ran before the log widget
        # existed).
        self.lane_model_var = tk.StringVar(value=LANE_MODELS[0][0])
        model_row = ttk.Frame(feats)
        model_row.grid(row=2, column=0, columnspan=2, sticky='ew',
                       pady=(4, 2))
        model_row.columnconfigure(1, weight=1)
        ttk.Label(model_row, text='Lane model:').grid(
            row=0, column=0, sticky='w')
        ttk.Combobox(model_row, textvariable=self.lane_model_var,
                     values=[d for d, _ in LANE_MODELS],
                     state='readonly', width=32).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))
        self.lkas_btn = ttk.Button(feats, text='LKAS: OFF', command=self.toggle_lkas)
        self.lkas_btn.grid(row=3, column=0, sticky='ew', pady=2)
        self.lkas_status_var = tk.StringVar(value='○ idle')
        self.lkas_status_lbl = ttk.Label(feats, textvariable=self.lkas_status_var,
                                          foreground='#888888', width=12)
        self.lkas_status_lbl.grid(row=3, column=1, sticky='w', padx=(6, 0))

        # Recorder. Writes whatever the camera widget is currently showing
        # (active source, decoded once per render tick) to a timestamped
        # .mp4 in <workspace>/recordings/.
        rec_frame = ttk.LabelFrame(left, text='Recording', padding=6)
        rec_frame.grid(row=9, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        rec_frame.columnconfigure(0, weight=1)
        self.record_btn = ttk.Button(rec_frame, text='● Record',
                                     command=self._toggle_recording)
        self.record_btn.grid(row=0, column=0, sticky='ew', pady=2)
        self.record_status_var = tk.StringVar(value='idle')
        self.record_status_lbl = ttk.Label(rec_frame,
                                            textvariable=self.record_status_var,
                                            foreground='#888888')
        self.record_status_lbl.grid(row=1, column=0, sticky='w')

        # Status + clear-log.
        self.status_var = tk.StringVar(value='Ready')
        ttk.Label(left, textvariable=self.status_var, foreground='#0044aa',
                  wraplength=240).grid(
            row=10, column=0, columnspan=2, sticky='w', pady=(10, 2))
        ttk.Button(left, text='Clear log', command=self._clear_log).grid(
            row=11, column=0, columnspan=2, sticky='ew', pady=2)

        # Right column: camera (top) + log (bottom).
        right = ttk.Frame(outer)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=2)

        self._cam_frame = ttk.LabelFrame(right, text='Live camera', padding=4)
        self._cam_frame.grid(row=0, column=0, sticky='nsew')
        self._cam_frame.columnconfigure(0, weight=1)
        self._cam_frame.rowconfigure(1, weight=1)

        # Source selector — switches between raw bridge feed and the YOLO /
        # UFLD / combined annotated debug images published by the perception
        # nodes. The right side of the bar shows live ego speed.
        src_bar = ttk.Frame(self._cam_frame)
        src_bar.grid(row=0, column=0, sticky='ew', pady=(0, 4))
        src_bar.columnconfigure(2, weight=1)
        ttk.Label(src_bar, text='Source:').grid(row=0, column=0, sticky='w')
        self._camera_source_var = tk.StringVar(value='Raw')
        ttk.Combobox(src_bar, textvariable=self._camera_source_var,
                     values=list(CAMERA_SOURCES.keys()),
                     state='readonly', width=18).grid(
            row=0, column=1, sticky='w', padx=(4, 0))
        self._camera_source_var.trace_add('write', lambda *_: self._refresh_cam_title())

        self.speed_var = tk.StringVar(value='Speed: —')
        ttk.Label(src_bar, textvariable=self.speed_var,
                  font=('Monospace', 11, 'bold'),
                  foreground='#0044aa').grid(
            row=0, column=2, sticky='e', padx=(8, 4))

        placeholder = ('waiting for first camera frame…'
                       if CAMERA_AVAILABLE
                       else f'camera disabled: {_camera_err}')
        # tk.Label width/height switch units depending on whether an image is
        # set. Hold the slot open with a pre-sized blank PhotoImage so the
        # widget has CAMERA_W × CAMERA_H pixels even before the first frame.
        self._placeholder_photo = tk.PhotoImage(width=CAMERA_W, height=CAMERA_H)
        self.camera_label = tk.Label(self._cam_frame, background='#222222', anchor='center',
                                      text=placeholder, foreground='#aaaaaa',
                                      image=self._placeholder_photo, compound='center')
        self.camera_label.image = self._placeholder_photo
        self.camera_label.grid(row=1, column=0, sticky='nsew')
        self._refresh_cam_title()

        log_frame = ttk.LabelFrame(right, text='Log', padding=4)
        log_frame.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_frame, height=10, width=80,
                                              font=('Monospace', 9), wrap='word')
        self.log.grid(row=0, column=0, sticky='nsew')

        # ── Permanent BEV panel (column 2 of outer) ─────────────────────
        # Always subscribed to /ADAS/ipm/debug_image — distinct from the
        # source-switchable camera widget on its left, so the operator
        # can see the BEV simultaneously with whichever camera source is
        # selected. Native IPM is 320×480, rendered here at BEV_W × BEV_H
        # with aspect preserved (see _render_bev_frame).
        self._bev_frame = ttk.LabelFrame(outer, text='BEV — /ADAS/ipm/debug_image',
                                         padding=4)
        self._bev_frame.grid(row=0, column=2, sticky='ns', padx=(8, 0))
        self._bev_placeholder_photo = tk.PhotoImage(width=BEV_W, height=BEV_H)
        bev_placeholder = ('waiting for IPM frame…'
                           if CAMERA_AVAILABLE
                           else f'BEV disabled: {_camera_err}')
        self.bev_label = tk.Label(self._bev_frame, background='#222222',
                                  anchor='center', text=bev_placeholder,
                                  foreground='#aaaaaa',
                                  image=self._bev_placeholder_photo,
                                  compound='center')
        self.bev_label.image = self._bev_placeholder_photo
        self.bev_label.grid(row=0, column=0, sticky='ns')

    # --------------------------------------------------------------------
    # Logging
    # --------------------------------------------------------------------
    def _log(self, msg: str):
        self.log.insert('end', msg + '\n')
        self.log.see('end')

    def _clear_log(self):
        self.log.delete('1.0', 'end')

    # --------------------------------------------------------------------
    # Live camera subscriber
    # --------------------------------------------------------------------
    def _start_camera_view(self):
        if not CAMERA_AVAILABLE:
            self._log(f'[ui] camera widget disabled: {_camera_err}')
            return
        try:
            rclpy.init()
        except RuntimeError:
            # Already initialised somewhere in this process.
            pass
        self.ros_node = TelemetryView()
        self.ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self.ros_node), daemon=True)
        self.ros_thread.start()
        # Schedule first render tick.
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)
        self._log(
            '[ui] subscribed to '
            + ', '.join(CAMERA_SOURCES.values())
            + f', {CMD_VEL_TOPIC}, {CMD_STEER_TOPIC}, {SPEED_TOPIC}')

    def _active_camera_topic(self) -> str:
        name = self._camera_source_var.get() if self._camera_source_var else 'Raw'
        return CAMERA_SOURCES.get(name, CAMERA_TOPIC)

    def _refresh_cam_title(self):
        # Show the active topic in the camera frame title so the user always
        # knows which feed they're looking at.
        topic = self._active_camera_topic()
        if hasattr(self, '_cam_frame'):
            self._cam_frame.configure(text=f'Live camera — {topic}')

    def _render_tick(self):
        if self.ros_node is not None:
            jpeg = self.ros_node.latest_jpegs.get(self._active_camera_topic())
            if jpeg is not None and jpeg is not self._last_rendered_jpeg:
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is not None:
                    self._last_bgr = bgr
                    self._render_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                    self._last_rendered_jpeg = jpeg
            if self._video_writer is not None and self._last_bgr is not None:
                self._record_frame(self._last_bgr)
            # BEV panel — independent of the source-selector camera. Same
            # decode-only-if-new pattern.
            bev_jpeg = self.ros_node.latest_bev_jpeg
            if bev_jpeg is not None and bev_jpeg is not self._last_rendered_bev_jpeg:
                arr = np.frombuffer(bev_jpeg, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is not None:
                    self._render_bev_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                    self._last_rendered_bev_jpeg = bev_jpeg
            self._refresh_status_dots()
            self._refresh_speed_label()
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)

    def _refresh_speed_label(self):
        v = self.ros_node.speed_mps if self.ros_node is not None else None
        if v is None or not self.ros_node.is_alive(SPEED_TOPIC):
            self.speed_var.set('Speed:  —  km/h')
        else:
            self.speed_var.set(f'Speed: {v * 3.6:5.1f} km/h')

    # --------------------------------------------------------------------
    # Video recorder
    # --------------------------------------------------------------------
    def _toggle_recording(self):
        if self._video_writer is None:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._last_bgr is None:
            self._log('[recorder] no camera frame yet — wait for the live '
                      'feed and try again')
            return
        rec_dir = ADAS_WK / 'recordings'
        rec_dir.mkdir(exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        source_tag = (self._camera_source_var.get()
                      .lower().replace(' ', '_').replace('(', '').replace(')', '').replace('+', '_'))
        self._video_path = rec_dir / f'adas_{ts}_{source_tag}.mp4'

        h, w = self._last_bgr.shape[:2]
        # mp4v is bundled with OpenCV's default build — no ffmpeg/H.264
        # licensing dance. Acceptable quality for a UI preview recording.
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(
            str(self._video_path), fourcc, float(CAMERA_UI_HZ), (w, h))
        if not writer.isOpened():
            self._log(f'[recorder] failed to open writer for '
                      f'{self._video_path} ({w}×{h}@{CAMERA_UI_HZ})')
            self._video_path = None
            return
        self._video_writer = writer
        self._video_size = (w, h)
        self.record_btn.configure(text='■ Stop')
        self.record_status_var.set(f'● REC  →  {self._video_path.name}')
        self.record_status_lbl.configure(foreground='#cc0000')
        self._log(f'[recorder] recording {w}×{h}@{CAMERA_UI_HZ} fps → '
                  f'{self._video_path}')

    def _record_frame(self, bgr: np.ndarray):
        # If the source switched and frame dims changed, resize to the
        # writer's original size — VideoWriter requires a constant frame size.
        h, w = bgr.shape[:2]
        if (w, h) != self._video_size:
            bgr = cv2.resize(bgr, self._video_size)
        try:
            self._video_writer.write(bgr)
        except Exception as e:
            self._log(f'[recorder] write failed: {e}; stopping recording')
            self._stop_recording()

    def _stop_recording(self):
        if self._video_writer is None:
            return
        try:
            self._video_writer.release()
        except Exception:
            pass
        path = self._video_path
        self._video_writer = None
        self._video_path = None
        self._video_size = None
        self.record_btn.configure(text='● Record')
        self.record_status_var.set(f'saved {path.name}' if path else 'idle')
        self.record_status_lbl.configure(foreground='#1a9b3c' if path else '#888888')
        self._log(f'[recorder] saved {path}')

    def _refresh_status_dots(self):
        # ACC is "active" when perception_node (publishes ACC debug image) AND
        # controller_node (publishes /Car_1/cmd_vel) are both heartbeating.
        # LKAS is "active" when lane_detection_node (publishes LKAS debug
        # image) AND stanley_node (publishes /Car_1/cmd_steer) are both up.
        node = self.ros_node
        if node is None:
            return
        perc_alive = node.is_alive(ACC_DEBUG_TOPIC)
        ctrl_alive = node.is_alive(CMD_VEL_TOPIC)
        lane_alive = node.is_alive(LKAS_DEBUG_TOPIC)
        stan_alive = node.is_alive(CMD_STEER_TOPIC)

        def label(perc_ok: bool, ctrl_ok: bool) -> tuple[str, str]:
            if perc_ok and ctrl_ok:
                return ('● active', '#1a9b3c')
            if perc_ok or ctrl_ok:
                return ('◐ partial', '#cc8800')
            return ('○ idle', '#888888')

        text, colour = label(perc_alive, ctrl_alive)
        self.acc_status_var.set(text)
        self.acc_status_lbl.configure(foreground=colour)
        text, colour = label(lane_alive, stan_alive)
        self.lkas_status_var.set(text)
        self.lkas_status_lbl.configure(foreground=colour)

    def _render_frame(self, rgb):
        # Resize to fit the camera widget (CAMERA_W × CAMERA_H), preserving
        # aspect ratio. If the user has expanded the window, grow up to the
        # actual label size.
        h, w = rgb.shape[:2]
        target_w = max(self.camera_label.winfo_width(), CAMERA_W)
        target_h = max(self.camera_label.winfo_height(), CAMERA_H)
        scale = min(target_w / w, target_h / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        if (new_w, new_h) != (w, h):
            rgb = cv2.resize(rgb, (new_w, new_h))
        img = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(img)
        self.camera_label.configure(image=photo, text='')
        self.camera_label.image = photo  # keep a reference; Tk won't otherwise

    def _render_bev_frame(self, rgb):
        # BEV panel — fixed target size BEV_W × BEV_H, aspect preserved.
        # IMPORTANT: do NOT use self.bev_label.winfo_width() here as the
        # target. The label resizes to fit each PhotoImage we set, which
        # nudges winfo_width() up; next tick that becomes the new target
        # and the panel grows again. Because the BEV column has no grid
        # weight (so the window doesn't clamp it), this feedback loop
        # snowballs frame-by-frame until the BEV consumes the whole UI.
        # Pinning the target to the constants kills the loop.
        h, w = rgb.shape[:2]
        scale = min(BEV_W / w, BEV_H / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        if (new_w, new_h) != (w, h):
            rgb = cv2.resize(rgb, (new_w, new_h),
                             interpolation=cv2.INTER_NEAREST)
        img = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(img)
        self.bev_label.configure(image=photo, text='')
        self.bev_label.image = photo  # keep a reference; Tk won't otherwise

    # --------------------------------------------------------------------
    # Subprocess helpers
    # --------------------------------------------------------------------
    def _stream(self, proc: subprocess.Popen, prefix: str):
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            self.root.after(0, self._log, f'[{prefix}] {line.rstrip()}')
        try:
            proc.stdout.close()
        except Exception:
            pass

    def _popen(self, cmd, cwd=None, source_ros=False, source_workspace=False,
               prefix='proc', extra_env=None) -> subprocess.Popen:
        """Launch a subprocess in its own process group. If `source_ros` is
        true, the command is run under `bash -c` after sourcing ROS (and
        optionally the ADAS workspace) so `rclpy` / `ros2 run` work.
        `extra_env` is merged on top of the current environment for the
        child."""
        env = None
        if extra_env:
            env = os.environ.copy()
            env.update(extra_env)
        if source_ros:
            parts = [f'source {shlex.quote(ROS_SETUP)}']
            if source_workspace and ADAS_INSTALL.exists():
                parts.append(f'source {shlex.quote(str(ADAS_INSTALL))}')
            parts.append('exec ' + ' '.join(shlex.quote(s) for s in cmd))
            shell_cmd = ' && '.join(parts)
            proc = subprocess.Popen(
                ['bash', '-c', shell_cmd], cwd=cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
            )
        else:
            proc = subprocess.Popen(
                cmd, cwd=cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
            )
        threading.Thread(target=self._stream, args=(proc, prefix),
                         daemon=True).start()
        return proc

    def _terminate(self, proc, label: str):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=4)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._log(f'[ui] {label} terminated')

    @staticmethod
    def _pkill(patterns):
        for p in patterns:
            subprocess.run(['pkill', '-f', p], check=False, capture_output=True)

    # --------------------------------------------------------------------
    # CARLA
    # --------------------------------------------------------------------
    def _pkill_all_carla(self):
        """Force-kill every CARLA server on the host, including one started
        by hand outside the UI. Without this, Start CARLA's new boot can't
        bind port 2000 because the orphan still owns it, AND we can't apply
        a new town because the orphan ignores our boot-map ini edit."""
        r = subprocess.run(['pkill', '-9', '-f', 'CarlaUE4'],
                           capture_output=True, text=True, check=False)
        if r.returncode == 0:
            self._log('[ui] pkill -9 CarlaUE4: killed running CARLA process(es)')
        time.sleep(2)  # let the OS release the RPC port

    def _on_town_change(self, _evt=None):
        # Town only takes effect at next CARLA boot (in-band load_world()
        # segfaults on this install).
        if self.carla_proc and self.carla_proc.poll() is None:
            self._log(f'[ui] town set to {self.town_var.get()} — '
                      'restart CARLA to load it.')

    def _write_boot_map(self):
        if not CARLA_INI.exists():
            self._log(f'[ui] warn: {CARLA_INI} missing; skipping boot-map edit')
            return
        try:
            n, wanted, observed = set_boot_map(CARLA_INI, self.town_var.get())
            self._log(f'[ui] boot map → {wanted} ({n} ini lines updated)')
            if observed != wanted:
                self._log('[ui] WARNING: ini did not retain the requested map.')
        except Exception as e:
            self._log(f'[ui] warn: could not edit {CARLA_INI}: {e}')

    def start_carla(self):
        # Always pkill first — even if `self.carla_proc` looks dead, there
        # may be a hand-launched CARLA bound to port 2000 that would silently
        # block our new boot AND ignore the boot-map ini we're about to write.
        self._pkill_all_carla()
        self._write_boot_map()
        port = int(self.port_var.get())
        cmd = [CARLA_SERVER, '-RenderOffScreen',
               f'-carla-rpc-port={port}',
               f'-quality-level={self.quality_var.get()}']
        self._log(f'$ (cd {CARLA_DIR} && {" ".join(cmd)})')
        self.carla_proc = self._popen(cmd, cwd=str(CARLA_DIR),
                                       source_ros=False, prefix='carla')
        self.status_var.set(
            f'CARLA starting on port {port} — town: {self.town_var.get()}')
        # Verify after CARLA has had time to boot — if the ini hack didn't
        # take (a known issue on this install per project md), the user
        # sees the mismatch in the log.
        self.root.after(15000, self._verify_loaded_map)

    def restart_carla(self):
        """Stop + Start in one click — useful after changing town/quality
        since those only apply on a fresh boot."""
        self.stop_carla()
        time.sleep(1)
        self.start_carla()

    def _verify_loaded_map(self):
        """Connect via the carla-env Python, read world.get_map().name, log
        it. If it doesn't match the requested town, flag it — map-switching
        is known to be non-deterministic on this install."""
        wanted = self.town_var.get()
        snippet = (
            "import carla, sys\n"
            "c = carla.Client('localhost', int(sys.argv[1])); c.set_timeout(5.0)\n"
            "print('[map] loaded:', c.get_world().get_map().name)\n"
        )
        self._run_carla_snippet(snippet, [self.port_var.get()], 'map')
        self._log(f'[ui] expected town: {wanted} (check [map] line above)')

    # --------------------------------------------------------------------
    # Weather / Traffic — applied via the carla-env Python over the RPC
    # connection. CARLA must already be running.
    # --------------------------------------------------------------------
    def _run_carla_snippet(self, snippet: str, args: list[str], prefix: str):
        """Run a one-shot carla-env snippet, fire-and-forget. For
        long-lived snippets the caller wants the proc handle back; use
        the returned value (None when carla-env is missing)."""
        if not CARLA_PYTHON.exists():
            self._log(f'[ui] carla python missing: {CARLA_PYTHON}')
            return None
        cmd = [str(CARLA_PYTHON), '-c', snippet] + args
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True,
        )
        threading.Thread(target=self._stream, args=(proc, prefix),
                         daemon=True).start()
        return proc

    def apply_weather(self):
        preset = self.weather_var.get()
        port = self.port_var.get()
        self._log(f'[ui] applying weather {preset} on port {port}')
        self._run_carla_snippet(_WEATHER_SNIPPET, [preset, port], 'weather')

    def spawn_traffic(self):
        n = self.traffic_var.get()
        port = self.port_var.get()
        # Kill the previous spawner first — the snippet is a forever
        # heartbeat loop, so without this every Spawn click would leave
        # an extra background process re-asserting autopilot on the
        # same vehicles. Replacing means the user can re-click Spawn
        # safely (e.g. to change N) without pile-up.
        self._terminate(self.npc_spawn_proc, 'NPC spawner')
        self.npc_spawn_proc = None
        self._log(f'[ui] spawning {n} NPC vehicles on port {port}')
        self.npc_spawn_proc = self._run_carla_snippet(
            _TRAFFIC_SPAWN_SNIPPET, [n, port], 'traffic')

    def clear_traffic(self):
        port = self.port_var.get()
        # Kill the heartbeat FIRST. If we destroyed the actors while it
        # was still running, the next 5-second tick would see them
        # already gone and re-spawn nothing, but during the destroy
        # window the heartbeat could race the clear and re-assert
        # autopilot on a half-destroyed actor. Killing first keeps the
        # teardown deterministic.
        self._terminate(self.npc_spawn_proc, 'NPC spawner')
        self.npc_spawn_proc = None
        self._log(f'[ui] clearing NPC traffic on port {port}')
        self._run_carla_snippet(_TRAFFIC_CLEAR_SNIPPET, [port], 'traffic')

    def list_spawns(self):
        port = self.port_var.get()
        self._log(f'[ui] listing spawn points on port {port}')
        self._run_carla_snippet(_LIST_SPAWNS_SNIPPET, [port], 'spawns')

    def stop_carla(self):
        # Always pkill — handles the case where CARLA was started outside the
        # UI and `self.carla_proc` is None.
        self._terminate(self.carla_proc, 'CARLA')
        self.carla_proc = None
        self._pkill_all_carla()
        self.status_var.set('CARLA stopped')

    # --------------------------------------------------------------------
    # Bridge (carlaAccSimTown.py)
    # --------------------------------------------------------------------
    def start_bridge(self):
        if self.bridge_proc and self.bridge_proc.poll() is None:
            self._log('[ui] Bridge already running')
            return
        policy = JUNCTION_POLICIES.get(
            self.junction_policy_var.get(), 'pp-takeover')
        # The Spinbox value comes through as a string — let argparse on the
        # bridge side parse it. If the user typed garbage, the bridge will
        # error with a clear message; no client-side validation needed.
        spawn_index = self.spawn_index_var.get().strip() or '0'
        cmd = [str(CARLA_PYTHON), str(BRIDGE_SCRIPT),
               '--junction-policy', policy,
               '--spawn-index', spawn_index]
        # Append --record only when the checkbox is ticked. Bridge defaults
        # to no recording (opt-in), so silence == no bag on disk.
        if self.rosbag_record_var.get():
            cmd.append('--record')
        self._log(f'$ (source ROS && cd {BRIDGE_DIR} && {" ".join(cmd)})')
        self.bridge_proc = self._popen(cmd, cwd=str(BRIDGE_DIR),
                                        source_ros=True, prefix='bridge')
        rec_note = ' + rosbag' if self.rosbag_record_var.get() else ''
        self.status_var.set(
            f'Bridge starting (spawn={spawn_index}, junction: {policy}{rec_note})')

    def stop_bridge(self):
        self._terminate(self.bridge_proc, 'Bridge')
        self.bridge_proc = None
        self.status_var.set('Bridge stopped')

    # --------------------------------------------------------------------
    # Foxglove bridge — ws://localhost:8765 for the Foxglove Studio app
    # --------------------------------------------------------------------
    def start_foxglove(self):
        if self.foxglove_proc and self.foxglove_proc.poll() is None:
            self._log('[ui] Foxglove bridge already running')
            return
        cmd = ['ros2', 'launch', 'foxglove_bridge',
               'foxglove_bridge_launch.xml']
        self._log(f'$ (source ROS && {" ".join(cmd)})')
        self.foxglove_proc = self._popen(cmd, source_ros=True,
                                          prefix='foxglove')
        self.status_var.set('Foxglove bridge starting on ws://localhost:8765')

    def stop_foxglove(self):
        self._terminate(self.foxglove_proc, 'Foxglove bridge')
        self.foxglove_proc = None
        self.status_var.set('Foxglove bridge stopped')

    # --------------------------------------------------------------------
    # start_acc.sh — launches all four ADAS nodes
    # --------------------------------------------------------------------
    def run_start_acc(self):
        if self.stack_proc and self.stack_proc.poll() is None:
            self._log('[ui] start_acc.sh already running')
            return
        cmd = ['./start_acc.sh', 'carla']
        self._log(f'$ (cd {ADAS_WK} && {" ".join(cmd)})')
        # start_acc.sh sources ROS itself.
        self.stack_proc = self._popen(cmd, cwd=str(ADAS_WK),
                                       source_ros=False, prefix='adas')
        # All four nodes are now alive — reflect that in the toggle state.
        self.acc_on = True
        self.lkas_on = True
        self._refresh_toggle_labels()
        self.status_var.set('ADAS stack running (start_acc.sh)')

    def stop_stack(self):
        self._terminate(self.stack_proc, 'start_acc.sh')
        self.stack_proc = None
        # start_acc.sh's children were spawned via `ros2 run` and don't share
        # our process group; sweep them up explicitly.
        self._pkill(['perception_node', 'controller_node',
                     'lane_detection_node', 'stanley_node'])
        for p in self.acc_procs + self.lkas_procs:
            self._terminate(p, 'subnode')
        self.acc_procs.clear()
        self.lkas_procs.clear()
        self.acc_on = False
        self.lkas_on = False
        self._refresh_toggle_labels()
        self.status_var.set('ADAS stack stopped')

    # --------------------------------------------------------------------
    # ACC / LKAS toggles
    # --------------------------------------------------------------------
    def _refresh_toggle_labels(self):
        self.acc_btn.configure(text=f'ACC: {"ON" if self.acc_on else "OFF"}')
        self.lkas_btn.configure(text=f'LKAS: {"ON" if self.lkas_on else "OFF"}')

    def toggle_acc(self):
        if self.acc_on:
            self._pkill(['perception_node', 'controller_node'])
            for p in self.acc_procs:
                self._terminate(p, 'acc-sub')
            self.acc_procs.clear()
            self.acc_on = False
            self._log('[ui] ACC OFF')
        else:
            # Look up the selected YOLO checkpoint in OBJECT_MODELS and
            # pass it to perception_node as -p model_filename:=<ref>.
            perc_cmd = ['ros2', 'run', 'perception', 'perception_node']
            model_ref = next((ref for name, ref in OBJECT_MODELS
                              if name == self.object_model_var.get()), None)
            if model_ref:
                perc_cmd += ['--ros-args', '-p',
                             f'model_filename:={model_ref}']
                self._log(f'[ui] ACC model: {self.object_model_var.get()}')
            self.acc_procs.append(self._popen(
                perc_cmd, cwd=str(ADAS_WK),
                source_ros=True, source_workspace=True,
                prefix='acc-perc'))
            self.acc_procs.append(self._popen(
                ['ros2', 'run', 'controller', 'controller_node',
                 '--ros-args', '-p', 'simulator:=carla'],
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
                prefix='acc-ctrl'))
            self.acc_on = True
            self._log('[ui] ACC ON')
        self._refresh_toggle_labels()

    def toggle_lkas(self):
        if self.lkas_on:
            self._pkill(['lane_detection_node', 'stanley_node'])
            for p in self.lkas_procs:
                self._terminate(p, 'lkas-sub')
            self.lkas_procs.clear()
            self.lkas_on = False
            self._log('[ui] LKAS OFF')
        else:
            # Look up the selected model in LANE_MODELS and pass it
            # to lane_detection_node. If the entry is missing (dropdown
            # value doesn't match any option — shouldn't happen since
            # it's readonly), we omit the flag and the node falls back
            # to its own default.
            perc_cmd = ['ros2', 'run', 'perception', 'lane_detection_node']
            model_ref = next((ref for name, ref in LANE_MODELS
                              if name == self.lane_model_var.get()), None)
            if model_ref:
                perc_cmd += ['--ros-args', '-p',
                             f'model_filename:={model_ref}']
                self._log(f'[ui] LKAS model: {self.lane_model_var.get()}')
            self.lkas_procs.append(self._popen(
                perc_cmd, cwd=str(ADAS_WK),
                source_ros=True, source_workspace=True,
                prefix='lkas-perc'))
            self.lkas_procs.append(self._popen(
                ['ros2', 'run', 'controller', 'stanley_node'],
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
                prefix='lkas-ctrl'))
            self.lkas_on = True
            self._log('[ui] LKAS ON')
        self._refresh_toggle_labels()


def main():
    root = tk.Tk()
    # Leave headroom around the 960×540 camera for the left controls + log.
    # Widened from 1320 → 1720 to fit the permanent BEV panel (BEV_W=360
    # + LabelFrame padding) to the right of the camera. Camera column
    # keeps its previous size — the new width is added, not redistributed.
    root.geometry('1720x820')
    app = ADASUI(root)

    def on_close():
        # Tear down what we own, in reverse-start order. Flush any in-flight
        # video recording first so the .mp4 has a valid trailer.
        app._stop_recording()
        # Kill the NPC heartbeat spawner before anything else — it's
        # the only one we created with start_new_session=True that has
        # no other lifecycle hook, so if we forget it it'll keep
        # running headless after the UI closes.
        app._terminate(app.npc_spawn_proc, 'NPC spawner')
        app.npc_spawn_proc = None
        app.stop_stack()
        app.stop_bridge()
        app.stop_carla()
        app.stop_foxglove()
        if app.ros_node is not None:
            try:
                app.ros_node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
