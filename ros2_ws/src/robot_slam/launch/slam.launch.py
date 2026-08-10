import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ekf_params_file(pkg):
    return os.path.join(pkg, 'config', 'ekf_local.yaml')


def _nav2_node(package_name, executable_name, node_name, params_file, use_sim_time, condition=None):
    return Node(
        package=package_name,
        executable=executable_name,
        name=node_name,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=condition,
    )


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

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
            'use_rviz',
            default_value='false',
            description='Launch RViz with the mapping layout',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch the Nav2 stack (controller, planner, etc). Not needed for Stage 1 mapping.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='ROS 2 parameters file for the Nav2 stack',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            description='ROS 2 parameters file for slam_toolbox',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg, 'rviz', 'mapping.rviz'),
            description='Path to the RViz config file',
        ),

        # Always-on: state publisher, EKF, scan relay, SLAM
        Node(
            package='xiao_serial_bridge',
            executable='scan_relay_node',
            name='scan_relay_node',
            output='screen',
            parameters=[{
                'target_beams': 720,
                'fov_deg': 250.0,
                'fov_center_deg': 0.0,
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[_ekf_params_file(pkg), {'use_sim_time': use_sim_time}],
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

        # Delay slam_toolbox 10s so EKF publishes odom→base_link TF before first scan lookup
        TimerAction(
            period=10.0,
            actions=[
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
            ],
        ),

        # Nav2 stack — only started when use_nav2:=true
        _nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapping',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
            condition=IfCondition(use_nav2),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
