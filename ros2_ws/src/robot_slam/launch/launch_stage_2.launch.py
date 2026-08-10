import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg, 'ros_data', 'maps', 'custom', 'sample_map.yaml'),
            description='Absolute path to the saved map YAML used for localization',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated clock',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start the Nav2 lifecycle stack',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'map_yaml': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            }.items(),
        ),
    ])
