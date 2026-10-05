from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='ir_dot_projector_ma',
            default_value='0.0',
            description='OAK-D Pro IR dot projector brightness in mA (0 = off, passive stereo only)',
        ),
        DeclareLaunchArgument(
            name='ir_flood_light_ma',
            default_value='0.0',
            description='OAK-D Pro IR flood light brightness in mA (0 = off)',
        ),
        Node(
            package='oakd_publisher',
            executable='oakd_publisher_node',
            name='oakd_publisher_node',
            output='screen',
            parameters=[{
                'ir_dot_projector_ma': LaunchConfiguration('ir_dot_projector_ma'),
                'ir_flood_light_ma': LaunchConfiguration('ir_flood_light_ma'),
            }],
        ),
    ])
