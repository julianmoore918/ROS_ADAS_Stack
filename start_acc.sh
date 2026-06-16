#!/bin/bash
# start_acc.sh — launches the ADAS stack (ACC + LKAS).
#
# Assumes:
#   1. CARLA server is already running (./CarlaUE4.sh or 0.10 launcher)
#   2. carla-ros-bridge is already up and publishing /Car_1/* topics
#
# Brings up:
#   ACC :   perception_node      (YOLO lead-vehicle detection)
#           controller_node      (ACC throttle/brake + LKAS steer injection)
#   LKAS:   lane_detection_node  (UFLD V2 lane polylines)
#           stanley_node         (Stanley lateral controller)

# ── Check argument ──────────────────────────────────────
if [ "$1" != "carla" ] && [ "$1" != "morai" ]; then
    echo "Usage: ./start_acc.sh [carla|morai]"
    echo "  Note: LKAS nodes are CARLA-tuned. In morai mode they still launch"
    echo "        but lane detection / Stanley behaviour has not been validated."
    exit 1
fi

SIMULATOR=$1
echo "[INFO] Starting ADAS stack for: $SIMULATOR"

# ── Source ROS 2 & workspace ────────────────────────────
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

# ── Launch ACC nodes ────────────────────────────────────
ros2 run perception perception_node &
PERCEPTION_PID=$!
echo "[INFO] ACC perception node started (PID $PERCEPTION_PID)"

ros2 run controller controller_node --ros-args -p simulator:=$SIMULATOR &
CONTROLLER_PID=$!
echo "[INFO] ACC controller node started (PID $CONTROLLER_PID)"

# ── Launch LKAS nodes ───────────────────────────────────
ros2 run perception lane_detection_node &
LANE_DETECTION_PID=$!
echo "[INFO] LKAS lane_detection_node started (PID $LANE_DETECTION_PID)"

ros2 run controller stanley_node &
STANLEY_PID=$!
echo "[INFO] LKAS stanley_node started (PID $STANLEY_PID)"

# ── Shutdown handler ────────────────────────────────────
PIDS="$PERCEPTION_PID $CONTROLLER_PID $LANE_DETECTION_PID $STANLEY_PID"
trap "echo; echo '[INFO] Shutting down ADAS stack…'; kill $PIDS 2>/dev/null; exit 0" SIGINT SIGTERM

wait