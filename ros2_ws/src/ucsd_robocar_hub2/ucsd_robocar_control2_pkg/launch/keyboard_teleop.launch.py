import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ctrl_pkg = 'ucsd_robocar_control2_pkg'

    config = os.path.join(
        get_package_share_directory(ctrl_pkg),
        'config',
        'keyboard_teleop_config.yaml')

    # Overrides the port in keyboard_teleop_config.yaml. /dev/vesc comes from
    # udev/99-rover.rules; the scripts in scripts/ fall back to /dev/ttyACM0.
    vesc_port = LaunchConfiguration('vesc_port')
    vesc_port_arg = DeclareLaunchArgument(
        'vesc_port',
        default_value='/dev/ttyACM1',
        description='VESC serial device')

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[config])

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[config, {'port': vesc_port}])

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[
            config,
            {
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': False,
                'use_servo_cmd_to_calc_angular_velocity': False,
            },
        ],
        remappings=[('/odom', '/vesc_odom')],
    )

    return LaunchDescription([
        vesc_port_arg,
        ackermann_to_vesc_node,
        vesc_driver_node,
        vesc_to_odom_node,
    ])
