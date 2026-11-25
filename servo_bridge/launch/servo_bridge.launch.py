from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    config = PathJoinSubstitution([FindPackageShare('servo_bridge'), 'config', 'servo_bridge.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the Arduino'),
        Node(
            package='servo_bridge',
            executable='servo_bridge_node',
            name='servo_bridge',
            output='screen',
            parameters=[config, {'port': port}],
        ),
    ])
