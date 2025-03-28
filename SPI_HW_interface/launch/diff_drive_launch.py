from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_config', default_value='diff_drive_controller.yaml'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': ''}, LaunchConfiguration('controller_config')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
    ])
