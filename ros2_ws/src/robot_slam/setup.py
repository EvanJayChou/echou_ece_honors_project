from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_slam'


def install_tree(source_root, destination_root):
    installed_files = []
    for root, _, files in os.walk(source_root):
        relative_root = os.path.relpath(root, source_root)
        if relative_root != '.':
            destination_dir = os.path.join(destination_root, relative_root)
        else:
            destination_dir = destination_root
        for file_name in files:
            installed_files.append((destination_dir, [os.path.join(root, file_name)]))
    return installed_files


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'ros_data', 'maps'), glob('ros_data/maps/custom/*')),
]
data_files.extend(
    install_tree(
        'urdf', os.path.join('share', package_name, 'urdf')
    )
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evanc',
    maintainer_email='evan.chou@live.com',
    description='SLAM and Nav2 launch package for robot_slam.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
