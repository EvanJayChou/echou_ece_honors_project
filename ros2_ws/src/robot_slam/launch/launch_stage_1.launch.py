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
        # Onboard-only stack: sensors, EKF, and the Nav2 controller. SLAM Toolbox and
        # the Nav2 global-planning side now run off-board — bring those up separately
        # via ros2_ws_offboard/src/offboard_nav/launch/offboard.launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'onboard.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_rviz': use_rviz,
            }.items(),
        ),
    ])
