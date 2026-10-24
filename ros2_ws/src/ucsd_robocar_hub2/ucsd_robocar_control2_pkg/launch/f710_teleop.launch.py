import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ctrl_pkg = 'ucsd_robocar_control2_pkg'

    config = os.path.join(
        get_package_share_directory(ctrl_pkg),
        'config',
        'f710_teleop_config.yaml')

    # Gamepad only. The VESC stack comes from keyboard_teleop.launch.py
    # (scripts/teleop.sh --joy starts both).
    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[config])

    f710_teleop_node = Node(
        package=ctrl_pkg,
        executable='f710_teleop_node',
        name='f710_teleop_node',
        output='screen',
        parameters=[config])

    return LaunchDescription([
        game_controller_node,
        f710_teleop_node,
    ])
