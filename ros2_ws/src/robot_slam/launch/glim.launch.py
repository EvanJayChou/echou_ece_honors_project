from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam.glim_config import assemble


def _glim_node(context):
    mode = 'gpu' if LaunchConfiguration('use_gpu').perform(context).lower() == 'true' else 'cpu'
    config_path = assemble(mode, f'/tmp/rover/glim_config_{mode}')

    return [Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'dump_path': LaunchConfiguration('dump_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # On Ctrl+C GLIM optimizes and writes the map dump before exiting; the
        # default 5 s SIGTERM escalation would kill it mid-write.
        sigterm_timeout='60',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gpu',
            default_value='true',
            description='GPU (VGICP_GPU) odometry and mapping; false selects the CPU configs',
        ),
        DeclareLaunchArgument(
            'dump_path',
            default_value='/tmp/rover/glim_dump',
            description='Where GLIM writes the map/trajectory dump on shutdown (overwritten each run)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (set true when replaying a bag with ros2 bag play --clock)',
        ),
        DeclareLaunchArgument(
            'publish_lidar_tf',
            default_value='false',
            description='Publish base_link -> livox_frame here. Leave false when onboard.launch.py '
                        '(robot_state_publisher) is running; it already publishes this from the URDF.',
        ),

        OpaqueFunction(function=_glim_node),

        # Same offset as lidar_joint in urdf/robot.urdf.xacro; keep the two in sync.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox_tf',
            arguments=['--x', '0.1', '--y', '0.0', '--z', '0.2',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
            condition=IfCondition(LaunchConfiguration('publish_lidar_tf')),
        ),
    ])
