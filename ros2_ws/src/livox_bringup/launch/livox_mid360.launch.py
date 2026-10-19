import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('livox_bringup')
    user_config_path = os.path.join(pkg, 'config', 'MID360_config.json')

    topic_name = LaunchConfiguration('topic_name')
    lidar_frame = LaunchConfiguration('lidar_frame')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    scan_height_min = LaunchConfiguration('scan_height_min')
    scan_height_max = LaunchConfiguration('scan_height_max')
    publish_freq = LaunchConfiguration('publish_freq')

    # xfer_format 2 = standard PCL PointXYZI PointCloud2 — required so that
    # pointcloud_to_laserscan below can consume it directly (formats 0/1 are
    # Livox-specific and are not plain sensor_msgs/PointCloud2).
    livox_driver_params = [
        {'xfer_format': 2},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': publish_freq},
        {'output_data_type': 0},
        {'frame_id': lidar_frame},
        {'user_config_path': user_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='topic_name',
            default_value='scan',
            description='Output 2D LaserScan topic name (matches the retired ldlidar/LD06 topic)',
        ),
        DeclareLaunchArgument(
            name='publish_freq',
            default_value='10.0',
            description='Point cloud frame rate (Hz). Not CPU-bound: the Mid-360 emits a fixed '
                        '~200k pts/s, so higher rates mean sparser frames and gappier 2D scans.',
        ),
        DeclareLaunchArgument(
            name='lidar_frame',
            default_value='livox_frame',
            description='Livox sensor frame_id',
        ),
        DeclareLaunchArgument(
            name='range_min',
            default_value='0.1',
            description='Minimum range (m) passed through to the LaserScan',
        ),
        DeclareLaunchArgument(
            name='range_max',
            default_value='40.0',
            description='Maximum range (m) passed through to the LaserScan',
        ),
        DeclareLaunchArgument(
            name='scan_height_min',
            default_value='-0.1',
            description='Lower Z bound (m) of the point-cloud slice flattened into the 2D scan',
        ),
        DeclareLaunchArgument(
            name='scan_height_max',
            default_value='0.1',
            description='Upper Z bound (m) of the point-cloud slice flattened into the 2D scan',
        ),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_driver_params,
        ),

        # Flattens the Mid-360's 3D point cloud (/livox/lidar) into a 2D
        # sensor_msgs/LaserScan on the same 'scan' topic the SLAM stack
        # (scan_relay_node -> slam_toolbox/amcl) previously consumed from ldlidar.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='livox_pointcloud_to_laserscan',
            output='screen',
            remappings=[('cloud_in', '/livox/lidar'), ('scan', topic_name)],
            parameters=[{
                'target_frame': lidar_frame,
                'transform_tolerance': 0.05,
                'min_height': scan_height_min,
                'max_height': scan_height_max,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,  # nominal; keep = 1 / publish_freq
                'range_min': range_min,
                'range_max': range_max,
                'use_inf': True,
            }],
        ),
    ])
