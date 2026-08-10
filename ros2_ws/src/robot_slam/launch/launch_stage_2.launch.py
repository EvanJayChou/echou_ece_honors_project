import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    mission_bridge_pkg = get_package_share_directory('mission_bridge')
    oakd_yolo_pkg = get_package_share_directory('oakd_yolo_safety')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map_yaml')
    enable_oakd_yolo = LaunchConfiguration('enable_oakd_yolo')
    oakd_yolo_blob_path = LaunchConfiguration('oakd_yolo_blob_path')
    oakd_stop_on_detection = LaunchConfiguration('oakd_stop_on_detection')

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
        DeclareLaunchArgument(
            'enable_oakd_yolo',
            default_value='false',
            description='Launch the Oak-D Lite YOLO safety monitor',
        ),
        DeclareLaunchArgument(
            'oakd_yolo_blob_path',
            default_value='',
            description='Path to a DepthAI YOLO blob file for the Oak-D safety monitor',
        ),
        DeclareLaunchArgument(
            'oakd_stop_on_detection',
            default_value='true',
            description='Publish zero Twist to /cmd_vel when a hazard is detected',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mission_bridge_pkg, 'launch', 'mission_bridge.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_yolo_pkg, 'launch', 'oakd_yolo_safety.launch.py')
            ),
            launch_arguments={
                'enable': enable_oakd_yolo,
                'blob_path': oakd_yolo_blob_path,
                'stop_on_detection': oakd_stop_on_detection,
            }.items(),
            condition=IfCondition(enable_oakd_yolo),
        ),
    ])
