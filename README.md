# ROS 2 Arduino and LiDAR Integration

This repository contains a ROS 2 launch file for integrating Arduino and LiDAR devices into a ROS 2 environment. The launch file configures and launches nodes to interact with Arduino and LiDAR hardware, enabling communication and data exchange with ROS 2.

## Features:

Configurable launch arguments for customizing serial port, baud rate, topics, channel type, frame ID, and more.
Launches nodes for writing data to an Arduino device (serial_data_writer), interacting with a LiDAR device (sllidar_node), and visualizing LiDAR scan data (rplidar_viewer).
Parameters passed to nodes allow for easy customization and reconfiguration.
Designed for seamless integration into ROS 2 projects for robotics and automation applications.

## Dependencies:
**1.** `ros2_serial_arduino` package for Arduino communication. [Click here to clone](https://github.com/dhanushshettigar/ros2_serial_arduino.git)

**2.** `sllidar_ros2 package` for LiDAR interaction. [Click here to clone](https://github.com/Slamtec/sllidar_ros2.git)

**3.** `ros2_rplidar_sub` package for LiDAR data visualization. [Click here to clone](https://github.com/dhanushshettigar/ros2_rplidar_sub.git)

## Clone the package in ros environment

**Step 1** Source the setup files

```bash
source /opt/ros/iron/setup.bash
```

**Step 2** Navigate to ros workspace source directory

```bash
cd home/ros/ros2_ws/src
```

**Step 3** Clone mapping_robot package from Github

Ensure you're still in the ros2_ws/src directory before you clone:

```bash
git clone https://github.com/dhanushshettigar/mapping_robot.git
```

**Step 4** Build mapping_robot package

From the root of your workspace (ros2_ws), you can now build mapping_robot package using the command:

```bash
cd ..
colcon build --symlink-install
```

## Launch Instructions

**Step 1** Make sure you have all the dependencies installed on your workspace.

**Step 2** Source the setup file of the workspace:

```bash
source install/setup.bash
```

**Step 3** Launch the nodes using the provided launch file:

```bash
ros2 launch mapping_robot mapper_launch.py
```

Monitor the output to ensure that the nodes are launched successfully and functioning as expected.
