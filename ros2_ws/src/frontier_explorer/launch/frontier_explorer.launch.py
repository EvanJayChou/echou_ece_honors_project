from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_start = LaunchConfiguration('auto_start')
    publish_end_on_complete = LaunchConfiguration('publish_end_on_complete')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Start exploring immediately without mission start command',
        ),
        DeclareLaunchArgument(
            'publish_end_on_complete',
            default_value='true',
            description='Publish mission end command when no frontiers remain',
        ),
        Node(
            package='frontier_explorer',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=[{
                'auto_start': auto_start,
                'publish_end_on_complete': publish_end_on_complete,
            }],
        ),
    ])
