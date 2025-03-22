from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Use mock hardware"),
        DeclareLaunchArgument("map", default_value="", description="Path to map file for Nav2")
    ]

    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    map_file = LaunchConfiguration("map")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("goku"), "urdf", "goku.urdf.xacro"]),
        " ",
        "use_mock_hardware:=", use_mock_hardware
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([FindPackageShare("goku"), "config", "diffbot_controllers.yaml"])
    slam_params = PathJoinSubstitution([FindPackageShare("goku"), "config", "slam_params.yaml"])
    nav2_params = PathJoinSubstitution([FindPackageShare("goku"), "config", "nav2_params.yaml"])
    ekf_params = PathJoinSubstitution([FindPackageShare("goku"), "config", "ekf.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen"
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )
    diffbot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"]
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params]
    )
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ]),
        launch_arguments={"slam_params_file": slam_params}.items()
    )
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ]),
        launch_arguments={"use_sim_time": "false", "params_file": nav2_params, "map": map_file}.items()
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui)
    )

    return LaunchDescription(declared_arguments + [
        control_node, robot_state_pub_node, joint_state_broadcaster, diffbot_controller,
        ekf_node, slam_node, nav2_node, rviz_node
    ])
