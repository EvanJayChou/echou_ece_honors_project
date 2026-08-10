from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyframe_bridge',
            executable='keyframe_publisher_node',
            name='keyframe_publisher_node',
            output='screen',
        ),
    ])
