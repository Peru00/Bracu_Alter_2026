# SLAM Launch Package

Complete SLAM system for RPLIDAR A3 using rf2o_laser_odometry and slam_toolbox.

## Overview

This package provides:
- **Mapping**: Create maps of your environment
- **Localization**: Navigate using a saved map

### Components
| Component | Purpose |
|-----------|---------|
| RPLIDAR A3 | 360° laser scanner (20Hz, 12m range) |
| rf2o_laser_odometry | Odometry from laser scan matching |
| slam_toolbox | SLAM and localization |

### TF Tree
```
map → odom → laser
      (slam)  (rf2o)
```

---

## Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select slam_launch --symlink-install
source install/setup.bash
```

### 2. Start Mapping
```bash
ros2 launch slam_launch complete_slam.launch.py
```

### 3. Visualize (in another terminal)
```bash
rviz2
# Add displays: LaserScan (/scan), Map (/map), TF
```

### 4. Save the Map
Once you've mapped your environment:
```bash
~/ros2_ws/src/slam_launch/scripts/save_map.sh my_room
```

### 5. Use Map for Localization
```bash
ros2 launch slam_launch localization.launch.py map:=~/ros2_ws/maps/my_room
```

---

## Detailed Usage

### Mapping Mode

Start SLAM to create a new map:

```bash
# Default (uses stable USB device path)
ros2 launch slam_launch complete_slam.launch.py

# With specific serial port
ros2 launch slam_launch complete_slam.launch.py serial_port:=/dev/ttyUSB0

# With custom config
ros2 launch slam_launch complete_slam.launch.py params:=/path/to/custom.yaml
```

**Topics Published:**
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Raw LIDAR data |
| `/odom_rf2o` | nav_msgs/Odometry | Laser odometry |
| `/map` | nav_msgs/OccupancyGrid | Current map |
| `/map_metadata` | nav_msgs/MapMetaData | Map info |

**Transforms Published:**
- `odom → laser` (rf2o_laser_odometry)
- `map → odom` (slam_toolbox)

### Saving Maps

Maps are saved in two formats:

1. **slam_toolbox format** (`.posegraph` + `.data`)
   - Used for localization with slam_toolbox
   - Contains full pose graph for continued SLAM

2. **Standard ROS format** (`.pgm` + `.yaml`)
   - Used with nav2_map_server
   - Standard occupancy grid image

```bash
# Save with default name (my_map)
~/ros2_ws/src/slam_launch/scripts/save_map.sh

# Save with custom name
~/ros2_ws/src/slam_launch/scripts/save_map.sh kitchen_map

# Maps saved to: ~/ros2_ws/maps/
```

**Manual save via service call:**
```bash
# Save slam_toolbox format
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/peru0002/ros2_ws/maps/my_map'}"
```

### Localization Mode

Use a saved map for robot localization (no new mapping):

```bash
# Use saved map
ros2 launch slam_launch localization.launch.py map:=~/ros2_ws/maps/my_room

# With all options
ros2 launch slam_launch localization.launch.py \
    map:=~/ros2_ws/maps/my_room \
    serial_port:=/dev/ttyUSB0
```

---

## Configuration

### slam.yaml (Mapping)

Key parameters in `config/slam.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping              # Create new map
    resolution: 0.05           # 5cm grid cells
    max_laser_range: 12.0      # RPLIDAR A3 max range
    throttle_scans: 3          # Process every 3rd scan
    do_loop_closing: true      # Enable loop closure
    minimum_travel_distance: 0.1  # Min movement for new scan
```

### localization.yaml (Localization)

Key parameters in `config/localization.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    mode: localization         # Use existing map
    do_loop_closing: false     # No new loop closures
    transform_publish_period: 0.05  # Faster TF updates
```

---

## Troubleshooting

### LIDAR Not Starting
```bash
# Check USB device
ls -la /dev/ttyUSB* /dev/serial/by-id/*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER
# Then logout and login
```

### No Map Published
```bash
# Check slam_toolbox state
ros2 lifecycle get /slam_toolbox

# Should be "active [3]". If not:
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### TF Errors
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom laser
```

### Check Topics
```bash
# List all topics
ros2 topic list

# Check scan rate
ros2 topic hz /scan

# View map metadata
ros2 topic echo /map_metadata --once
```

---

## File Structure

```
slam_launch/
├── config/
│   ├── slam.yaml           # Mapping configuration
│   └── localization.yaml   # Localization configuration
├── launch/
│   ├── complete_slam.launch.py   # Mapping launch
│   └── localization.launch.py    # Localization launch
├── scripts/
│   └── save_map.sh         # Map saving script
├── rviz/
│   └── slam.rviz           # RViz configuration
├── CMakeLists.txt
└── package.xml
```

---

## Dependencies

- `rplidar_ros` - RPLIDAR driver
- `rf2o_laser_odometry` - Laser odometry
- `slam_toolbox` - SLAM and localization
- `nav2_map_server` (optional) - Standard map format saving
