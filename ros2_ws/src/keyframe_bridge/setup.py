from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'keyframe_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evanc',
    maintainer_email='evan.chou@live.com',
    description='Onboard keyframe selection and streaming to the off-board 3DGS mapper.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyframe_publisher_node = keyframe_bridge.keyframe_publisher_node:main',
        ],
    },
)
