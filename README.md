# ROS2 ADAS Stack (ACC + LKAS)

Minimal ROS 2 implementation of an ADAS system combining Adaptive Cruise
Control (ACC) and Lane-Keeping Assist (LKAS).

The stack consists of four nodes split across two packages:

ACC:

- **perception_node** – YOLO-based lead vehicle detection and distance estimation
- **controller_node** – ACC controller publishing throttle / brake

LKAS:

- **lane_detection_node** – UFLD V2 lane detection, publishes ego-left / ego-right polylines in the vehicle frame
- **stanley_node** – Stanley lateral controller, publishes a normalised steer command

Longitudinal and lateral control travel on **separate topics** to the CARLA
bridge:

- ACC `controller_node` owns `/Car_1/cmd_vel` (`linear.x` = throttle,
  `linear.y` = brake).
- LKAS `stanley_node` owns `/Car_1/cmd_steer` (`Float32`, normalised steer
  ∈ [-1, 1]).

The bridge subscribes to both and merges them into a single
`carla.VehicleControl` per command callback. When LKAS isn't running, steer
stays at the bridge's last-seen value (0 on startup) and the system behaves
as pure ACC; when ACC isn't running, throttle/brake stay at 0.

## Workspace Structure

    ROS_ADAS_Stack/
     ├── start_acc.sh               ← simulator startup script (launches all four nodes)
     └── src/
         ├── perception/
         │   ├── perception_node.py        (ACC — YOLO)
         │   ├── lane_detection_node.py    (LKAS — UFLD V2)
         │   └── models/                   (weights: best.pt, UFLD_best.pth, RLD_best.pth)
         └── controller/
             ├── controller_node.py        (ACC — throttle / brake)
             └── stanley_node.py           (LKAS — Stanley lateral controller, publishes /Car_1/cmd_steer)

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- ultralytics
- opencv-python
- numpy < 2
- cv_bridge

Install Python dependencies:

    python3 -m pip install ultralytics opencv-python "numpy<2"

## Build

    cd ~/acc_ws
    colcon build
    source install/setup.bash

## Run

The startup script in the workspace root launches all four nodes (ACC + LKAS).
It sources ROS 2 and the workspace automatically and accepts a simulator argument:

    cd ~/workspace/03_ADAS_WK/ROS_ADAS_Stack
    ./start_acc.sh carla       # for CARLA simulator
    ./start_acc.sh morai       # for MORAI simulator  (LKAS not validated for morai)

Press **Ctrl+C** to shut down all nodes cleanly at the same time.

**Prerequisite:** the CARLA server and the CARLA↔ROS bridge must already be
running before `start_acc.sh` is launched. The script only orchestrates the ADAS
nodes; it does not start the simulator or the bridge.

To run nodes manually in separate terminals:

    # ACC
    ros2 run perception perception_node
    ros2 run controller controller_node --ros-args -p simulator:=carla
    # LKAS
    ros2 run perception lane_detection_node
    ros2 run controller stanley_node

## Simulator Parameter

The controller node selects the correct message type for `/Car_1/vehicle/speed`
based on the `simulator` parameter:

| Simulator | Message type | Default |
|-----------|-------------|---------|
| `carla`   | `std_msgs/msg/Float64` | ✓ |
| `morai`   | `example_interfaces/msg/Float64` | |

## ROS Topics

Simulator interface:

    /Car_1/camera/front/compressed     sensor_msgs/msg/CompressedImage    ← camera input
    /Car_1/vehicle/speed               Float64 (type depends on simulator) ← ego speed
    /Car_1/cmd_vel                     geometry_msgs/msg/Twist             ← linear.x = throttle,
                                                                             linear.y = brake
    /Car_1/cmd_steer                   std_msgs/msg/Float32                ← normalised steer ∈ [-1, 1], positive = right

ACC internal topics:

    /ACC/lead_vehicle_distance         std_msgs/msg/Float32    ← distance to lead vehicle [m]
    /ACC/lead_vehicle_confidence       std_msgs/msg/Float32    ← YOLO detection confidence [0–1]
    /ACC/target_speed                  std_msgs/msg/Float32    ← target speed [km/h]  (Foxglove slider)

LKAS internal topics:

    /LKAS/ego_lane_left                nav_msgs/msg/Path           ← ego-left polyline (vehicle frame, REP 103: X fwd, Y left)
    /LKAS/ego_lane_right               nav_msgs/msg/Path           ← ego-right polyline

Debug / visualization:

    /ACC/perception/debug_image        sensor_msgs/msg/CompressedImage    ← annotated YOLO detections
    /LKAS/perception/debug_image       sensor_msgs/msg/CompressedImage    ← annotated lane polylines

## Node Overview

### ACC

#### Perception Node (`perception_node.py`)

Subscribes:

    /Car_1/camera/front/compressed

Publishes:

    /ACC/lead_vehicle_distance         closest detected vehicle [m], inf when none
    /ACC/lead_vehicle_confidence       YOLO confidence of closest detection, 0.0 when none
    /ACC/perception/debug_image        annotated image for Foxglove visualization

Function:

- YOLO vehicle detection (car, truck, bus, motorcycle)
- Optional ROI mask (toggle `USE_ROI` flag at top of file, default: off)
- Ego-lane filtering — only detects vehicles within ±20% of image centre
- Bounding box height based distance estimation using focal length
- Minimum confidence threshold (`MIN_CONFIDENCE = 0.5`)
- Publishes `inf` / `0.0` when no vehicle is detected (`PUBLISH_INF` flag)
- Single-line terminal status output (no scroll)
- Debug image streamed to Foxglove via `/ACC/perception/debug_image`

Config flags at the top of the file:

    USE_ROI        = False   # enable/disable ROI mask
    MIN_CONFIDENCE = 0.5     # detections below this are ignored
    PUBLISH_INF    = True    # publish inf on no-detection (set False to suppress)

#### Controller Node (`controller_node.py`)

Subscribes:

    /Car_1/vehicle/speed               ego speed (type selected by simulator parameter)
    /ACC/lead_vehicle_distance         distance to lead vehicle
    /ACC/target_speed                  target cruising speed [km/h]

Publishes:

    /Car_1/cmd_vel                     throttle (linear.x) and brake (linear.y)

Control modes:

| Mode | Condition | Behaviour |
|------|-----------|-----------|
| CRUISE | No lead vehicle detected | Proportional speed controller toward target speed |
| ACC | Lead vehicle in range | PD-based distance controller |
| EMERGENCY | Lead vehicle < 3 m | Immediate full brake |

Control law (ACC mode):

    d_desired = d0 + T_gap * v_ego
    a = k_p * (d_lead - d_desired) + k_d * closing_rate

Default parameters:

    target_speed       = 13.9 m/s  (≈ 50 km/h, overridable via /ACC/target_speed)
    d0                 = 5.0 m     (standstill gap)
    T_gap              = 1.5 s     (time gap)
    k_p                = 1.2       (proportional gain)
    k_d                = 0.8       (derivative gain)
    emergency_distance = 3.0 m

### LKAS

#### Perception Node (`lane_detection_node.py`)

Subscribes:

    /Car_1/camera/front/compressed

Publishes:

   /LKAS/ego_lane_left
   /LKAS/ego_lane_right
   /LKAS/perception/debug_image        annotated image for Foxglove visualization

Function:
- Calls trained UFLD V2 model for lane detection and outputs left and right lane as polylines.

#### Controller Node (`stanley_node.py`)

Subscribes:

    /Car_1/vehicle/speed               ego speed (type selected by simulator parameter)
    /LKAS/ego_lane_left
    /LKAS/ego_lane_right

Publishes:

   /Car_1/cmd_steer

Function:

```
delta = e_head + atan2(k * e_lat, v + eps)
steer = clamp(delta / max_steer_angle, -1.0, 1.0)
```

| Symbol | Meaning | 0.9.16 | 0.10.0 |
|--------|---------|--------|--------|
| `k` | cross-track gain | `0.5` | **`1.0`** (UE5.5 vehicles feel sluggish at 0.5) |
| `v` | current vehicle speed (m/s) | runtime | runtime |
| `eps` | speed regulariser to avoid division by zero | `0.5` | `0.5` |
| `max_steer_angle` | normalising scale for steer output (rad) | `≈ 1.22` (70°) | **`≈ 0.70`** (40° — tighter scale → more direct feel) |

> Note: `stanley_node.py` in this ROS stack uses the **0.9.16** column values
> (`STANLEY_K = 0.5`, `MAX_STEER_RAD = math.radians(70)`), matching the CARLA
> version the bridge talks to. The 0.10.0 column is the tuning used in the
> standalone `lkas_validate_0.10.0.py` script in `02_UFLD_V2/`.


## Setting Target Speed via Foxglove

A Variable Slider panel in Foxglove can adjust the target speed at runtime:

1. Add a **Variable Slider** panel — variable name: `target_speed`, min: `0`, max: `130`, step: `1`
2. Add a **Publish** panel — topic: `/ACC/target_speed`, datatype: `std_msgs/Float32`, message: `{ "data": "$target_speed" }`

The controller converts the incoming km/h value to m/s internally.

## Foxglove Visualization

Connect Foxglove to `ws://localhost:8765` (foxglove-bridge).
Recommended panels:

| Panel | Topic | Notes |
|-------|-------|-------|
| Image | `/ACC/perception/debug_image` | Annotated YOLO detections with distance labels |
| Image | `/Car_1/camera/front/compressed` | Raw camera feed |
| Plot  | `/ACC/lead_vehicle_distance` | Distance over time |
| Plot  | `/ACC/lead_vehicle_confidence` | Detection confidence over time |
| Plot  | `/Car_1/vehicle/speed` | Ego speed over time |
| Variable Slider + Publish | `/ACC/target_speed` | Set target speed at runtime |
