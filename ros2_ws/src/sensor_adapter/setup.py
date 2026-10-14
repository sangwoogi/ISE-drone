import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ISE-drone',
    maintainer_email='drknu12@gmail.com',
    description='Common sensor topics (/camera/*) from Gazebo (ros_gz_bridge) or real cameras. Phase 3.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_adapter_node = sensor_adapter.sensor_adapter_node:main'
        ],
    },
)
