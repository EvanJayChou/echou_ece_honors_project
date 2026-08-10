import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nav2_node(package_name, executable_name, node_name, params_file, use_sim_time):
    return Node(
        package=package_name,
        executable=executable_name,
        name=node_name,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg, 'ros_data', 'maps', 'custom', 'sample_map.yaml'),
            description='Absolute path to the saved map YAML used for AMCL localization',
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
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='ROS 2 parameters file for the Nav2 stack',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg, 'rviz', 'localization.rviz'),
            description='Path to the RViz config file',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': open(
                        os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
                    ).read(),
                },
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=[
                os.path.join(pkg, 'config', 'ekf_local.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            parameters=[
                os.path.join(pkg, 'config', 'nav2_params.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
            ],
        ),

        _nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time),
        _nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time),
        _nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time),
        _nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time),
        _nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time),
        _nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time),
        _nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
