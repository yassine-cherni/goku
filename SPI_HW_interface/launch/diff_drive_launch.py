from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('diff_drive_hw_interface')
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'diff_drive_controller.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_config',
            default_value=controller_config,
            description='Path to controller configuration file'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': ''}, LaunchConfiguration('controller_config')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
