# Ouster Bridge

A ROS2 package that bridges Ouster LiDAR localization data to ROS topics, enabling simultaneous operation of Ouster ROS driver and ouster-cli localization.

## Overview

This package provides:
- **CSV TF Bridge**: Converts ouster-cli localization CSV output to ROS TF transforms and odometry
- **Launch Files**: Orchestrates ouster-cli localization with ROS integration
<<<<<<< HEAD

=======
>>>>>>> 7377a740670d1e3f4598f54e229c08251d067cde

## Features

- Real-time localization pose publishing to `/odom` topic
- TF tree broadcasting for robot localization
- CSV-based pose bridging from ouster-cli

## Dependencies

- ROS2 (tested on Humble)
- ouster-cli (Ouster SDK)
- Ouster ROS driver
- Python packages: `rclpy`, `tf2_ros`, `nav_msgs`, `geometry_msgs`

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/ouster_bridge.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ouster_bridge
source install/setup.bash
```

## Usage

### Prerequisites

1. Have an Ouster LiDAR sensor configured and accessible
2. Create a map using ouster-cli SLAM:
```bash
ouster-cli source YOUR_SENSOR_IP slam save survey.osf
ouster-cli source survey.osf save map.ply
```

### Running Localization Only

```bash
ros2 launch ouster_bridge localize_with_bridge.launch.py
```

This will:
- Start ouster-cli localization against your map
- Bridge the pose data to ROS `/odom` topic
- Publish TF transforms


### Configuration

Edit the launch file parameters:
- `sensor_ip`: Your Ouster sensor IP address
- `map_file`: Path to your .ply map file  
- `csv_file`: Path where localization poses will be saved

## Topics Published

- `/odom` (nav_msgs/Odometry): Robot odometry from localization
- `/tf` (tf2_msgs/TFMessage): Transform tree updates
- `/tf_static` (tf2_msgs/TFMessage): Static transforms

## Package Structure

```
ouster_bridge/
├── launch/
│   └── localize_with_bridge.launch.py    # Main launch file
├── ouster_bridge/
│   ├── __init__.py
│   └── csv_tf_bridge.py                  # CSV to ROS bridge node
├── package.xml                           # Package manifest
├── setup.py                             # Python package setup
└── README.md                            # This file
```

## Troubleshooting

### No localization data
- Ensure your map file exists and is valid
- Check that the sensor is reachable at the specified IP
- Verify ouster-cli is installed and accessible

### TF/Odometry not publishing
- Check that the CSV file is being created and updated
- Verify the csv_tf_bridge node is running
- Ensure file permissions allow reading the CSV

### Conflicts with ROS driver
- The package is not designed to work alongside the Ouster ROS driver
- Both are not able to connect to the sensor simultaneously in most cases

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Authors

- Hassan


## Authors

- Hassan

## Acknowledgments

- Ouster for the excellent LiDAR hardware and SDK
- ROS community for the robotics framework
