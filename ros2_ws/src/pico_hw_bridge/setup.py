from setuptools import find_packages, setup

package_name = 'pico_hw_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fzc17',
    maintainer_email='kennethubert17@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pico_hw_server = pico_hw_bridge.pico_hw_server:main',
            'led_state_monitor = pico_hw_bridge.led_state_monitor:main',
        ],
    },
)
