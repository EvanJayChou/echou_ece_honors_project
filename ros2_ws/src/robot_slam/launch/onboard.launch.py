import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# Nav2 lifecycle nodes, in the order the lifecycle manager brings them up.
NAV2_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


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
    frontier_explorer_pkg = get_package_share_directory('frontier_explorer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_explorer = LaunchConfiguration('use_explorer')
    explorer_auto_start = LaunchConfiguration('explorer_auto_start')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    if_nav2 = IfCondition(use_nav2)

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
            'use_slam',
            default_value='true',
            description='Run slam_toolbox on the Jetson (default). false only when SLAM runs '
                        'off-board (scripts/offboard.sh slam) — never run both.',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch the full Nav2 stack (planner, BT navigator, behaviors, controller, '
                        'costmaps, velocity smoother) and the cmd_vel -> ackermann_cmd converter.',
        ),
        DeclareLaunchArgument(
            'use_explorer',
            default_value='false',
            description='Launch frontier_explorer (Q1 baseline). Only takes effect with use_nav2:=true.',
        ),
        DeclareLaunchArgument(
            'explorer_auto_start',
            default_value='false',
            description='Start exploring immediately instead of waiting for /mission/control "start"',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_onboard.yaml'),
            description='ROS 2 parameters file for Nav2',
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

        # Always-on nodes: scan relay, state publisher, EKF pose estimate.
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

        # Fuses VESC vx with the OAK-D Pro IMU yaw rate (/oakd/imu); see ekf_local.yaml.
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

        # SLAM (on by default). Delayed so the EKF publishes odom→base_link TF
        # before slam_toolbox's first scan lookup. The EKF starts emitting as soon
        # as /vesc_odom arrives (<1 s on the Orin Nano); 5 s leaves margin.
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

        # Nav2 — only started when use_nav2:=true.
        # Standard wiring: controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel.
        _nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time,
                   condition=if_nav2, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        _nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time,
                   condition=if_nav2),
        _nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time,
                   condition=if_nav2),
        _nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time,
                   condition=if_nav2, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        _nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time,
                   condition=if_nav2,
                   remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        _nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time,
                   condition=if_nav2),
        _nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time,
                   condition=if_nav2),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': NAV2_NODES,
            }],
            condition=if_nav2,
        ),

        # Ackermann base: ackermann_to_vesc listens on /ackermann_cmd, not /cmd_vel.
        Node(
            package='robot_slam',
            executable='cmd_vel_to_ackermann_node',
            name='cmd_vel_to_ackermann_node',
            output='screen',
            condition=if_nav2,
        ),

        # Q1 exploration baseline — replaced by uncertainty-driven NBV goals in Q2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(frontier_explorer_pkg, 'launch', 'frontier_explorer.launch.py')
            ),
            launch_arguments={'auto_start': explorer_auto_start}.items(),
            condition=IfCondition(PythonExpression([
                "'", use_nav2, "'.lower() == 'true' and '", use_explorer, "'.lower() == 'true'",
            ])),
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
