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


def _nav2_node(package_name, executable_name, node_name, params_file, use_sim_time,
               condition=None, remappings=None):
    return Node(
        package=package_name,
        executable=executable_name,
        name=node_name,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=condition,
        remappings=remappings or [],
    )


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
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
            description='Automatically start the onboard Nav2 lifecycle stack',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz with the mapping layout',
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Run slam_toolbox on the Jetson. Leave false when SLAM runs off-board '
                        '(ros2_ws_offboard offboard.launch.py) — never run both.',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch the onboard Nav2 controller stack (controller_server, local_costmap, '
                        'velocity_smoother) and the cmd_vel -> ackermann_cmd converter.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_onboard.yaml'),
            description='ROS 2 parameters file for the onboard Nav2 controller',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            description='ROS 2 parameters file for onboard slam_toolbox (use_slam:=true)',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg, 'rviz', 'mapping.rviz'),
            description='Path to the RViz config file',
        ),

        # Always-on onboard nodes: scan relay, state publisher, EKF pose estimate.
        # Sensor drivers (livox_bringup, oakd_publisher) and the VESC driver are
        # started separately by the scripts in scripts/.
        Node(
            package='robot_slam',
            executable='scan_relay_node',
            name='scan_relay_node',
            output='screen',
            parameters=[{
                # ~542 beams survive the 270° front crop of the 723-beam Livox scan; the
                # rear 90° faces the rover body / mount.
                'target_beams': 540,
                'fov_deg': 270.0,
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

        # Optional onboard SLAM. Delayed so the EKF publishes odom→base_link TF
        # before slam_toolbox's first scan lookup. The EKF starts emitting as soon
        # as /vesc_odom arrives (<1 s on the Orin Nano); 5 s leaves margin. The Pi 5
        # needed 10 s.
        TimerAction(
            period=5.0,
            condition=IfCondition(use_slam),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam_params_file': slam_params_file,
                    }.items(),
                ),
            ],
        ),

        # Onboard Nav2 controller — only started when use_nav2:=true.
        # This is the tight closed-loop piece that must stay low-latency, so it
        # stays on the Jetson even though the global planner/BT that drives it is off-board.
        # Standard Nav2 wiring: controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel.
        _nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time,
                   condition=IfCondition(use_nav2),
                   remappings=[('cmd_vel', 'cmd_vel_nav')]),
        _nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time,
                   condition=IfCondition(use_nav2),
                   remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        # Ackermann base: ackermann_to_vesc listens on /ackermann_cmd, not /cmd_vel.
        Node(
            package='robot_slam',
            executable='cmd_vel_to_ackermann_node',
            name='cmd_vel_to_ackermann_node',
            output='screen',
            condition=IfCondition(use_nav2),
        ),

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
