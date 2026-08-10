from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_publisher',
            executable='oakd_publisher_node',
            name='oakd_publisher_node',
            output='screen',
        ),
    ])
