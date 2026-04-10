from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("use_gazebo", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"))
    declared_arguments.append(DeclareLaunchArgument("baud_rate", default_value="115200"))
    declared_arguments.append(DeclareLaunchArgument("feed_rate", default_value="800"))
    declared_arguments.append(DeclareLaunchArgument("abs_motion", default_value="true"))

    # Initialize Arguments
    use_gazebo = LaunchConfiguration("use_gazebo")
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    feed_rate = LaunchConfiguration("feed_rate")
    abs_motion = LaunchConfiguration("abs_motion")

    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    pkg_arctos_description = FindPackageShare("arctos_description")
    robot_controllers = PathJoinSubstitution([pkg_arctos_description, "config", "arctos_controllers.yaml"])

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_arctos_description, "urdf", "arctos.xacro"]),
        " use_gazebo:=", use_gazebo,
        " port_name:=", port_name,
        " baud_rate:=", baud_rate,
        " feed_rate:=", feed_rate,
        " abs_motion:=", abs_motion
    ])
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution([FindPackageShare("arctos_bringup"), "rviz", "OL_robot.rviz"])
    gz_bridge_config_file = PathJoinSubstitution([pkg_arctos_description, "config", "ros_gz_bridge.yaml"])

    # Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={"gz_args": "-r -s -v 3 empty.sdf"}.items(),
        condition=IfCondition(use_gazebo)
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "arctos_robot", "-allow_renaming", "true"],
        condition=IfCondition(use_gazebo)
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_file}],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
        condition=UnlessCondition(use_gazebo)
    )

    # --- SPAWNERS ---

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_gazebo}]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_gazebo}]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_gripper_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_gazebo}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # --- EVENT HANDLERS (Sequence Management) ---

    # 1. Start Broadcaster after Gazebo Spawn (Sim) or Control Node (HW)
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_gazebo)
    )

    # 2. Start Arm Controller after Broadcaster
    delay_robot_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # 3. Start Gripper Controller after Arm Controller
    delay_gripper_controller_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes_to_start = [
        robot_state_publisher,
        control_node,
        rviz_node,
        # Gazebo path
        gazebo,
        gz_spawn_entity,
        ros_gz_bridge,
        delay_broadcaster_after_spawn,
        # Hardware path (Broadcaster starts immediately)
        # 1. Joint State Broadcaster (Always needed)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            condition=UnlessCondition(use_gazebo)
        ),

        # 2. Arm Controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arctos_arm_controller"],
        ),

        # 3. Gripper Controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arctos_gripper_controller"],
        ),
        # Shared Path
        delay_robot_controller_after_broadcaster,
        delay_gripper_controller_after_robot_controller
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)