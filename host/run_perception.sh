#!/usr/bin/env bash
# Phase 15 (perception layer): boots the camera-equipped Gazebo world
# headless, bridges the camera to real ROS2 topics, and runs the
# perception node -- everything host/perception_node.py's own header
# comment documents running by hand, in one script.
#
# Usage:
#   bash host/setup_perception_venv.sh   # once
#   bash host/run_perception.sh
#   # elsewhere:
#   ros2 topic echo /detections vision_msgs/msg/Detection2DArray
#   rviz2   # Image display on /detections_image shows boxes drawn live
set -uo pipefail

cd "$(dirname "$0")/.."
VENV_DIR="${PERCEPTION_VENV:-$HOME/perception_venv}"
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "error: $VENV_DIR not found -- run 'bash host/setup_perception_venv.sh' first" >&2
    exit 1
fi

GZ_PID=""
BRIDGE_PID=""
PERCEPTION_PID=""
cleanup() {
    echo "-- tearing down gz sim, bridge, and perception node --" >&2
    [ -n "$PERCEPTION_PID" ] && kill -9 "$PERCEPTION_PID" 2>/dev/null
    [ -n "$BRIDGE_PID" ] && kill -9 "$BRIDGE_PID" 2>/dev/null
    [ -n "$GZ_PID" ] && kill -9 "$GZ_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "== booting camera-equipped world headless =="
setsid gz sim -s -r host/gazebo/diff_drive_camera.sdf > /tmp/perception_gz.log 2>&1 &
GZ_PID=$!
sleep 5

echo "== bridging /camera and /camera_info to real ROS2 topics =="
setsid ros2 run ros_gz_bridge parameter_bridge \
    "/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
    > /tmp/perception_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 3

echo "== starting perception node (YOLOv8n, first run downloads weights) =="
setsid bash -c "source '$VENV_DIR/bin/activate' && exec python3 host/perception_node.py" \
    > /tmp/perception_node.log 2>&1 &
PERCEPTION_PID=$!

echo "== running. Try, elsewhere: =="
echo "     ros2 topic echo /detections vision_msgs/msg/Detection2DArray"
echo "     ros2 topic hz /detections_image"
echo "     rviz2   # Image display on /detections_image shows live boxes"
echo "     ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \\"
echo "       '{linear: {x: 0.3}}' -r 10   # drive toward the walking actor"
echo "Ctrl-C to stop everything."
wait "$PERCEPTION_PID"
