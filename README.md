# drone_fly_circle

A ROS 2 Python package that commands a PX4 drone to fly in a circular trajectory using offboard control via Micro XRCE Agent.

## Table of Contents

- [Overview](#overview)  
- [Prerequisites](#prerequisites)  
- [Installation](#installation)  
- [Usage](#usage)  
- [Parameters](#parameters)  
- [Troubleshooting](#troubleshooting)  
- [License](#license)  

## Overview

This package  
1. Launches PX4 SITL (via `make px4_sitl gz_x500`)  
2. Starts a Micro XRCE Agent (`udp4`, port `8888`)  
3. Runs the `drone_fly_circle` node, which switches the vehicle into offboard mode and publishes trajectory setpoints for a circular flight.

## Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble (tested)  
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) cloned into `~/PX4-Autopilot`  
- [`micro-ros-agent`](https://github.com/micro-ROS/micro-ROS-Agent) installed and available on `PATH`  
- **px4_msgs.msg ROS package** built and sourced in your workspace  
- Python 3 dependencies (will be pulled in by `rosdep`)

> **Note:** You must have the `px4_msgs.msg` package in your ROS 2 workspace prior to building this package.

## Installation

1. Source your ROS 2 installation:
   ```
   source /opt/ros/foxy/setup.bash
   ```

2. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your_org/drone_flycircle.git
   ```

3. Ensure `px4_msgs` is present and built in the same workspace:
   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

4. Source your workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

To start the simulation, Micro XRCE Agent, and the circle-flight node in one go:

```bash
ros2 launch drone_fly_circle drone_control_xrce.launch.py
```

This will:

- Build and run PX4 SITL using Gazebo (`gz_x500`)  
- Launch a Micro XRCE Agent listening on UDP port 8888  
- Start the `drone_fly_circle` node that commands the drone to fly a circle of radius 20 m at 0.5 rad/s, starting at −5 m altitude.

### Customization

You can edit parameters in `drone_fly_circle/drone_fly_circle.py`:

- `radius` — circle radius (meters)  
- `angular_velocity` — angular speed (rad/s)  
- `starting_attitude` — altitude (meters)  

## Parameters

| Parameter           | Default | Description                       |
|---------------------|---------|-----------------------------------|
| `radius`            | `20`    | Desired circle radius in meters   |
| `angular_velocity`  | `0.5`   | Angular speed in radians per sec. |
| `starting_attitude` | `-5.0`  | Altitude offset from ground (m)   |

## Troubleshooting

- **PX4 SITL fails to build**  
  Make sure `~/PX4-Autopilot` exists and you have run `make px4_sitl gazebo` at least once.
- **Micro XRCE Agent not found**  
  Install via `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888` or follow the micro-ROS installation guide.
- **Missing `px4_msgs`**  
  Clone and build the [px4_msgs ROS 2 package](https://github.com/PX4/px4_msgs) in your workspace.

## License

GNU GPL v3