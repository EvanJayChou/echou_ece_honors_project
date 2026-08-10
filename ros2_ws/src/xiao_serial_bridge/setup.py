from setuptools import setup

package_name = 'xiao_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Chou',
    maintainer_email='e3chou@ucsd.edu',
    description='ROS 2 serial bridge for XIAO nRF52840 Sense IMU odometry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = xiao_serial_bridge.serial_bridge_node:main',
            'scan_relay_node = xiao_serial_bridge.scan_relay_node:main',
        ],
    },
)
