import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    params_file = LaunchConfiguration('params_file')
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
            description='Automatically start the onboard Nav2 lifecycle stack',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz with the mapping layout',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch the onboard Nav2 controller stack (controller_server, local_costmap, velocity_smoother).',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_onboard.yaml'),
            description='ROS 2 parameters file for the onboard Nav2 controller',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg, 'rviz', 'mapping.rviz'),
            description='Path to the RViz config file',
        ),

        # Always-on onboard nodes: raw sensor relay, state publisher, EKF pose estimate.
        # SLAM Toolbox, the Nav2 global-planning side, and frontier_explorer now run
        # off-board — see ros2_ws_offboard/src/offboard_nav/launch/offboard.launch.py.
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

        # Onboard Nav2 controller — only started when use_nav2:=true.
        # This is the tight closed-loop piece that must stay low-latency, so it
        # stays on the Pi even though the global planner/BT that drives it is off-board.
        _nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),
        _nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time,
                   condition=IfCondition(use_nav2)),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_onboard',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
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
