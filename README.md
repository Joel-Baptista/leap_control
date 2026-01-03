# Leap Control
> **ROS 2 package for controlling the Leap Hand platform**

[![ROS Distro](https://img.shields.io/badge/ROS2-Humble-blue.svg)](#)
[![Build Status](https://img.shields.io/github/actions/workflow/status/<org>/<repo>/ci.yaml?branch=main)](#)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)](LICENSE)
[![Documentation](https://img.shields.io/badge/docs-online-success.svg)](<docs-url>)

---

## Overview

This repository contains the **control and integration package** for the **[Robot Name]** system.  
It implements hardware interfaces, control nodes, configuration files, and simulation assets for deployment in both **real hardware** and **Gazebo/RViz simulation**.

Typical use cases:
- Real-time motion control (velocity, position, or torque)
- Integration with [`ros2_control`](https://control.ros.org/)
- Sensor feedback publication (`/odom`, `/joint_states`)
- Teleoperation and autonomous mode switching
- Simulation and visualization support

---

## Architecture

[package_name]/
├── CMakeLists.txt
├── package.xml


---

## Installation

### Prerequisites
- OS: Ubuntu 24
- ROS2 Kilted


Some specific use cases may require the installation of the following python packages:
- [urdf2webots](https://pypi.org/project/urdf2webots/)

These python dependencies are not required to utilize the repository, but may be required to develop new code.

### Build
```bash
# ROS 2
source /opt/ros/kilted/setup.sh
mkdir -p ~/leap_ws/src
cd ~/ros_ws/src
git clone git@github.com:Joel-Baptista/leap_control.git
cd ~/leap_ws
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Convert URDF to Proto

After changing the URDF file, we need to run some commands to update the description for Webots.
To update the proto file, run the following commands from the root of the repository:
```bash
xacro leap_description/assets/leap_hand/robot.urdf.xacro > leap_description/urdf/robot.urdf
python -m urdf2webots.importer --input=leap_description/urdf/robot.urdf --output=leap_sim/protos --normal --init-pos="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rm leap_sim/protos/Leap_textures -rf
cd ~/leap_ws
colcon build --symlink-install
```
Make sure you have [urdf2webots](https://pypi.org/project/urdf2webots/) installed and the virtual environment sourced.

### Launch

### Run Node

## Simulation

## Testing

## References

- URDFs and .stl files in "leap_description/assets" imported from [here](https://github.com/leap-hand/LEAP_Hand_Sim)
- Leap Hand comes from the published worked described [here](https://v1.leaphand.com/)

