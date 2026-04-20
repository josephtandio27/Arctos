# ros2_arctos

## Project Overview
![Demo Screencast](Video/Screencast from 2026-04-20 14-34-31.gif)

Open-loop arctos robotic arm using Moveit 2 with serial communication to Arduino Mega. Wiring follows the official guide, but without hall effect sensors. Make sure to change the port_name in view.arctos.launch.py or in arctos.xacro (for demo.launch.py). The initial position of the robot is defined as in the CAD file, so make sure to position the robot in the same way before running either view.arctos.launch.py or demo.launch.py. To change the speed and send GRBL command, publish to topic /set_feed_rate and /send_grbl_command, respectively.

This project contains the ROS 2 packages for the Arctos robotic system, including:
- **arctos_bringup**: Launch files and configuration for bringing up the robot system
- **arctos_description**: URDF/XACRO files describing the robot's physical structure
- **arctos_hardware_interface**: Hardware interface components for communicating with robot hardware
- **arctos_moveit_config**: MoveIt 2 configuration for motion planning and control

## Getting Started

### Prerequisites

- ROS 2 Jazzy
- Colcon build system
- Required dependencies (see package.xml)

### Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone <repository-url> src/Arctos
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the setup files:
   ```bash
   source install/setup.bash
   ```

### Usage

Refer to individual package documentation for specific usage instructions:
- See `arctos_bringup` for launching the robot system
- See `arctos_description` for robot model information
- See `arctos_hardware_interface` for hardware integration details
- See `arctos_moveit_config` for motion planning configuration

Reference: 
- https://github.com/cr0Kz/ros2_arctos/tree/feature/motor_driver
- https://arctosrobotics.com/docs/#wiring

### Moving the robot with topic
ros2 topic pub --once /arctos_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint'], points: [{positions: [0.0, -0.3, 0.264, -0.296, 0.389, 0.1], time_from_start: {sec: 3, nanosec: 0}}]}"

### Moving the robot with action
ros2 action send_goal /arctos_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint'],
    points: [
      {
        positions: [0.0, 0.0, 0.0, 0.0, 0.00, 0.0],
        time_from_start: {sec: 5, nanosec: 0}
      }
    ]
  }
}"
