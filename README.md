# ros2_arctos
Reference: https://github.com/cr0Kz/ros2_arctos/tree/feature/motor_driver

Open-loop arctos robotic arm

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
