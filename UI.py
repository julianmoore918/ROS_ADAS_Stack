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
CARLA_SERVER  = './CarlaUE4.sh'
CARLA_INI     = Path('/home/sirius/CARLA_0.9.16/CarlaUE4/Config/DefaultEngine.ini')
CARLA_PYTHON  = Path('/home/sirius/CARLA_0.9.16/carla-env/bin/python3')
BRIDGE_DIR    = Path('/home/sirius/workspace/carlaaccsim')
BRIDGE_SCRIPT = BRIDGE_DIR / 'carlaAccSimTown.py'
START_ACC_SH  = ADAS_WK / 'start_acc.sh'
ROS_SETUP     = '/opt/ros/humble/setup.bash'

CAMERA_TOPIC  = '/Car_1/camera/front/compressed'
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
import carla, random, sys
n = int(sys.argv[1])
port = int(sys.argv[2])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
random.shuffle(spawn_points)
vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*')
               if int(bp.get_attribute('number_of_wheels')) == 4]
tm = client.get_trafficmanager(port + 6000)
tm.set_global_distance_to_leading_vehicle(2.5)
tm.set_synchronous_mode(False)
spawned = 0
for sp in spawn_points:
    if spawned >= n:
        break
    bp = random.choice(vehicle_bps)
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'npc')
    actor = world.try_spawn_actor(bp, sp)
    if actor is None:
        continue
    actor.set_autopilot(True, tm.get_port())
    spawned += 1
print(f'[traffic] spawned {spawned}/{n} NPCs')
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


# --------------------------------------------------------------------------
# ROS camera subscriber
# --------------------------------------------------------------------------
class CameraView(Node):
    """Subscribes to the bridge's compressed camera topic and hands frames
    (numpy RGB) to a callback."""

    def __init__(self, on_frame):
        super().__init__('adas_ui_camera_view')
        self.on_frame = on_frame
        self.create_subscription(CompressedImage, CAMERA_TOPIC, self._cb, 10)

    def _cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.on_frame(rgb)


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
        self.acc_procs: list[subprocess.Popen] = []             # when toggled independently
        self.lkas_procs: list[subprocess.Popen] = []

        self.acc_on = False
        self.lkas_on = False

        # ROS subscriber for the camera widget.
        self.ros_node: CameraView | None = None
        self.ros_thread: threading.Thread | None = None

        # Frame throttling.
        self._latest_rgb = None
        self._render_after = None

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
        ttk.Button(procs, text='Run start_acc.sh', command=self.run_start_acc).grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop ADAS Stack', command=self.stop_stack).grid(
            row=4, column=0, columnspan=2, sticky='ew', pady=2)

        # Feature toggles.
        feats = ttk.LabelFrame(left, text='Features', padding=6)
        feats.grid(row=8, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        feats.columnconfigure(0, weight=1)
        self.acc_btn = ttk.Button(feats, text='ACC: OFF', command=self.toggle_acc)
        self.acc_btn.grid(row=0, column=0, sticky='ew', pady=2)
        self.lkas_btn = ttk.Button(feats, text='LKAS: OFF', command=self.toggle_lkas)
        self.lkas_btn.grid(row=1, column=0, sticky='ew', pady=2)

        # Status + clear-log.
        self.status_var = tk.StringVar(value='Ready')
        ttk.Label(left, textvariable=self.status_var, foreground='#0044aa',
                  wraplength=240).grid(
            row=9, column=0, columnspan=2, sticky='w', pady=(10, 2))
        ttk.Button(left, text='Clear log', command=self._clear_log).grid(
            row=10, column=0, columnspan=2, sticky='ew', pady=2)

        # Right column: camera (top) + log (bottom).
        right = ttk.Frame(outer)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=2)

        cam_frame = ttk.LabelFrame(right, text=f'Live camera — {CAMERA_TOPIC}', padding=4)
        cam_frame.grid(row=0, column=0, sticky='nsew')
        cam_frame.columnconfigure(0, weight=1)
        cam_frame.rowconfigure(0, weight=1)
        placeholder = ('waiting for first camera frame…'
                       if CAMERA_AVAILABLE
                       else f'camera disabled: {_camera_err}')
        # tk.Label width/height switch units depending on whether an image is
        # set. Hold the slot open with a pre-sized blank PhotoImage so the
        # widget has CAMERA_W × CAMERA_H pixels even before the first frame.
        self._placeholder_photo = tk.PhotoImage(width=CAMERA_W, height=CAMERA_H)
        self.camera_label = tk.Label(cam_frame, background='#222222', anchor='center',
                                      text=placeholder, foreground='#aaaaaa',
                                      image=self._placeholder_photo, compound='center')
        self.camera_label.image = self._placeholder_photo
        self.camera_label.grid(row=0, column=0, sticky='nsew')

        log_frame = ttk.LabelFrame(right, text='Log', padding=4)
        log_frame.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_frame, height=10, width=80,
                                              font=('Monospace', 9), wrap='word')
        self.log.grid(row=0, column=0, sticky='nsew')

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
        self.ros_node = CameraView(self._on_frame_received)
        self.ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self.ros_node), daemon=True)
        self.ros_thread.start()
        # Schedule first render tick.
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)
        self._log(f'[ui] subscribed to {CAMERA_TOPIC} for live view')

    def _on_frame_received(self, rgb):
        # Called from rclpy spin thread — just stash the latest frame; the
        # Tk after()-loop picks it up at CAMERA_UI_HZ.
        self._latest_rgb = rgb

    def _render_tick(self):
        rgb = self._latest_rgb
        if rgb is not None:
            self._render_frame(rgb)
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)

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
               prefix='proc') -> subprocess.Popen:
        """Launch a subprocess in its own process group. If `source_ros` is
        true, the command is run under `bash -c` after sourcing ROS (and
        optionally the ADAS workspace) so `rclpy` / `ros2 run` work."""
        if source_ros:
            parts = [f'source {shlex.quote(ROS_SETUP)}']
            if source_workspace and ADAS_INSTALL.exists():
                parts.append(f'source {shlex.quote(str(ADAS_INSTALL))}')
            parts.append('exec ' + ' '.join(shlex.quote(s) for s in cmd))
            shell_cmd = ' && '.join(parts)
            proc = subprocess.Popen(
                ['bash', '-c', shell_cmd], cwd=cwd,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
            )
        else:
            proc = subprocess.Popen(
                cmd, cwd=cwd,
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
        if not CARLA_PYTHON.exists():
            self._log(f'[ui] carla python missing: {CARLA_PYTHON}')
            return
        cmd = [str(CARLA_PYTHON), '-c', snippet] + args
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True,
        )
        threading.Thread(target=self._stream, args=(proc, prefix),
                         daemon=True).start()

    def apply_weather(self):
        preset = self.weather_var.get()
        port = self.port_var.get()
        self._log(f'[ui] applying weather {preset} on port {port}')
        self._run_carla_snippet(_WEATHER_SNIPPET, [preset, port], 'weather')

    def spawn_traffic(self):
        n = self.traffic_var.get()
        port = self.port_var.get()
        self._log(f'[ui] spawning {n} NPC vehicles on port {port}')
        self._run_carla_snippet(_TRAFFIC_SPAWN_SNIPPET, [n, port], 'traffic')

    def clear_traffic(self):
        port = self.port_var.get()
        self._log(f'[ui] clearing NPC traffic on port {port}')
        self._run_carla_snippet(_TRAFFIC_CLEAR_SNIPPET, [port], 'traffic')

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
        cmd = [str(CARLA_PYTHON), str(BRIDGE_SCRIPT)]
        self._log(f'$ (source ROS && cd {BRIDGE_DIR} && {" ".join(cmd)})')
        self.bridge_proc = self._popen(cmd, cwd=str(BRIDGE_DIR),
                                        source_ros=True, prefix='bridge')
        self.status_var.set('Bridge starting')

    def stop_bridge(self):
        self._terminate(self.bridge_proc, 'Bridge')
        self.bridge_proc = None
        self.status_var.set('Bridge stopped')

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
            self.acc_procs.append(self._popen(
                ['ros2', 'run', 'perception', 'perception_node'],
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
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
            self.lkas_procs.append(self._popen(
                ['ros2', 'run', 'perception', 'lane_detection_node'],
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
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
    root.geometry('1320x820')
    app = ADASUI(root)

    def on_close():
        # Tear down what we own, in reverse-start order.
        app.stop_stack()
        app.stop_bridge()
        app.stop_carla()
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
