import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    pkg = get_package_share_directory('offboard_nav')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    frontier_explorer_pkg = get_package_share_directory('frontier_explorer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
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
            default_value=os.path.join(pkg, 'config', 'nav2_params_offboard.yaml'),
            description='ROS 2 parameters file for the off-board Nav2 servers',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            description='ROS 2 parameters file for slam_toolbox',
        ),

        # Occupancy-grid mapping — consumes /scan + odom/tf published from the rover
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items(),
        ),

        # Nav2 global-planning side: bt_navigator, planner, smoother, behaviors,
        # waypoint follower, global costmap. controller_server / local_costmap /
        # velocity_smoother stay onboard (see robot_slam/launch/onboard.launch.py).
        _nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time),
        _nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time),
        _nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time),
        _nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time),
        _nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_offboard',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
        ),

        # Q1 exploration baseline — replaced by uncertainty-driven NBV selection in Q2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(frontier_explorer_pkg, 'launch', 'frontier_explorer.launch.py')
            ),
        ),
    ])
