from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _get_robot_descriptions(description_dir):
    """Get robot description parameters from xacro and srdf files."""
    urdf_file = PathJoinSubstitution([
        description_dir,
        'urdf',
        'arctos.xacro'
    ])
    srdf_file = PathJoinSubstitution([
        description_dir,
        'config',
        'arctos.srdf'
    ])

    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(
        robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(['cat ', srdf_file])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    return robot_description, robot_description_semantic


def _get_config_files(description_dir):
    """Get paths to configuration files."""
    robot_description_kinematics = PathJoinSubstitution([
        description_dir,
        'config',
        'kinematics.yaml'
    ])

    moveit_planners = PathJoinSubstitution([
        description_dir,
        'config',
        'moveit_planners.yaml'
    ])

    hardware_config = PathJoinSubstitution([
        description_dir,
        'config',
        'ros2_controllers.yaml'
    ])

    moveit_controllers = PathJoinSubstitution([
        description_dir,
        'config',
        'moveit_controllers.yaml'
    ])

    return robot_description_kinematics, moveit_planners, hardware_config, moveit_controllers


def generate_launch_description():
    """Generate launch description for Arctos robot with MoveIt and ROS2 Control."""
    # Directories
    description_dir = FindPackageShare('arctos_description')

    # Robot descriptions
    robot_description, robot_description_semantic = _get_robot_descriptions(
        description_dir)

    # Configuration files
    robot_description_kinematics, moveit_planners, hardware_config, moveit_controllers = _get_config_files(
        description_dir)

    print("Hardware config path:", hardware_config)

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, robot_description_semantic],
        namespace="",
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[hardware_config, robot_description],
        output='screen',
    )

    # Controller Spawners
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    trajectory_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    "joint_trajectory_controller",
                    "--param-file", hardware_config,
                    "-c", "/controller_manager"
                ],
            )
        ]
    )

    # MoveIt move_group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers,
            robot_description_kinematics,
            {'publish_robot_description_semantic': True},
            moveit_planners,
        ],
        namespace="",
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [description_dir, 'config', 'moveit.rviz'])
    rviz_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )]
    )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_spawner,
        trajectory_spawner,
        move_group_node,
        rviz_node,
    ])
