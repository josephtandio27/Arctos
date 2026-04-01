from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
            description="Simulation in Gazebo or real hardware"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyUSB0",
            description="Serial port name for Arduino Mega"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for serial communication with Arduino Mega"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "feed_rate",
            default_value="800",
            description="Speed of the coordinated vector through space. "
            "Every motor starts and stops at exactly the same time."
            "The total combined speed of all axes moving together will not exceed this value."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "abs_motion",
            default_value="true",
            description="Use absolute motion (G90) or relative motion (G91)"
        )
    )
    # Initialize Arguments
    use_gazebo = LaunchConfiguration("use_gazebo")
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    feed_rate = LaunchConfiguration("feed_rate")
    abs_motion = LaunchConfiguration("abs_motion")

    gz_launch_path = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])
    pkg_arctos_description = FindPackageShare("arctos_description")
    robot_controllers = PathJoinSubstitution(
        [
            pkg_arctos_description,
            "config",
            "arctos_controller.yaml",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    pkg_arctos_description,
                    "urdf",
                    "arctos_urdf.xacro",
                ]
            ),
            " use_gazebo:=", use_gazebo,
            " port_name:=", port_name,
            " baud_rate:=", baud_rate,
            " feed_rate:=", feed_rate,
            " abs_motion:=", abs_motion
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arctos_bringup"), "rviz", "view_robot.rviz"]
    )

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
        launch_arguments={"gz_args": "-r -v 3 empty.sdf"}.items(),
        condition=IfCondition(use_gazebo)
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "arctos_robot",
            "-allow_renaming", "true",
        ],
        condition=IfCondition(use_gazebo)
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("/forward_position_controller/commands", "/position_commands"),
        ],
        output="screen",
        condition=UnlessCondition(use_gazebo)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",],
        parameters=[{"use_sim_time": use_gazebo}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_controller", 
                   "-c", "/controller_manager",
                   "--param-file", robot_controllers],
        parameters=[{"use_sim_time": use_gazebo}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    delay_joint_state_broadcaster_after_gz_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        control_node,
        delay_joint_state_broadcaster_after_gz_spawn,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
