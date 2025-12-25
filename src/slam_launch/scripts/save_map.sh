#!/bin/bash
# Save map from slam_toolbox
# Usage: ./save_map.sh [map_name]
#
# This saves both:
#   - .posegraph (for slam_toolbox localization)
#   - .pgm + .yaml (standard ROS map format, if nav2_map_server installed)

set -e

MAP_NAME=${1:-my_map}
MAP_DIR="${HOME}/ros2_ws/maps"

mkdir -p "$MAP_DIR"

echo "================================================"
echo "Saving map: $MAP_DIR/$MAP_NAME"
echo "================================================"

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Method 1: Save via slam_toolbox service (creates .posegraph and .data files)
echo ""
echo "[1/2] Saving slam_toolbox format (.posegraph)..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$MAP_DIR/$MAP_NAME'}" || echo "Warning: serialize_map failed"

# Method 2: Save standard map format via nav2_map_server (if available)
echo ""
echo "[2/2] Saving standard format (.pgm + .yaml)..."
if command -v ros2 &>/dev/null && ros2 pkg list 2>/dev/null | grep -q nav2_map_server; then
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/${MAP_NAME}_std" --ros-args -p save_map_timeout:=10.0 || \
        echo "Warning: map_saver failed (map may not be published)"
else
    echo "Skipped: nav2_map_server not installed"
fi

echo ""
echo "================================================"
echo "Map saved to: $MAP_DIR/"
ls -la "$MAP_DIR/" | grep "$MAP_NAME" || echo "(No files found with that name)"
echo "================================================"
echo ""
echo "To use this map for localization:"
echo "  ros2 launch slam_launch localization.launch.py map:=$MAP_DIR/$MAP_NAME"
