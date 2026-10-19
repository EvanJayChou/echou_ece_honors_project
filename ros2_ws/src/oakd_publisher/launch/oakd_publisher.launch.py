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
        DeclareLaunchArgument(
            name='rgb_width',
            default_value='1280',
            description='Color/depth output width (px)',
        ),
        DeclareLaunchArgument(
            name='rgb_height',
            default_value='720',
            description='Color/depth output height (px)',
        ),
        DeclareLaunchArgument(
            name='fps',
            default_value='15.0',
            description='Color/depth frame rate',
        ),
        DeclareLaunchArgument(
            name='enable_imu',
            default_value='true',
            description='Publish the OAK-D Pro onboard IMU on /oakd/imu (feeds the EKF)',
        ),
        DeclareLaunchArgument(
            name='imu_rate_hz',
            default_value='200',
            description='IMU accelerometer/gyro report rate',
        ),
        DeclareLaunchArgument(
            name='imu_rotation_source',
            default_value='calibration',
            description="'calibration' = IMU axes from device EEPROM, 'manual' = use imu_rpy",
        ),
        DeclareLaunchArgument(
            name='gyro_bias_calibration_s',
            default_value='5.0',
            description='Seconds of stationary gyro samples averaged into the bias at startup',
        ),
        Node(
            package='oakd_publisher',
            executable='oakd_publisher_node',
            name='oakd_publisher_node',
            output='screen',
            parameters=[{
                'ir_dot_projector_ma': LaunchConfiguration('ir_dot_projector_ma'),
                'ir_flood_light_ma': LaunchConfiguration('ir_flood_light_ma'),
                'rgb_width': LaunchConfiguration('rgb_width'),
                'rgb_height': LaunchConfiguration('rgb_height'),
                'fps': LaunchConfiguration('fps'),
                'enable_imu': LaunchConfiguration('enable_imu'),
                'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
                'imu_rotation_source': LaunchConfiguration('imu_rotation_source'),
                'gyro_bias_calibration_s': LaunchConfiguration('gyro_bias_calibration_s'),
            }],
        ),
    ])
