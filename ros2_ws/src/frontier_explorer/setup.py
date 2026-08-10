from glob import glob

from setuptools import find_packages, setup

package_name = 'frontier_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/frontier_explorer']),
        ('share/frontier_explorer', ['package.xml']),
        ('share/frontier_explorer/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evanc',
    maintainer_email='evan.chou@live.com',
    description='Frontier-based autonomous exploration node for Stage 1 mapping.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = frontier_explorer.frontier_explorer_node:main',
        ],
    },
)
