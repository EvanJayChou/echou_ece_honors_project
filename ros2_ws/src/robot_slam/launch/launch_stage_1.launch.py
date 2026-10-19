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
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')

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
            default_value='false',
            description='Run slam_toolbox onboard instead of off-board',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Run the onboard Nav2 controller + cmd_vel->ackermann converter',
        ),
        # Onboard-only stack: scan relay, EKF, and optionally the Nav2 controller.
        # SLAM Toolbox and the Nav2 global-planning side run off-board by default —
        # bring those up via ros2_ws_offboard/src/offboard_nav/launch/offboard.launch.py,
        # or pass use_slam:=true to map on the Jetson for a quick self-contained test.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'onboard.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_rviz': use_rviz,
                'use_slam': use_slam,
                'use_nav2': use_nav2,
            }.items(),
        ),
    ])
