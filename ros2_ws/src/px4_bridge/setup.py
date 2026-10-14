import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ISE-drone',
    maintainer_email='drknu12@gmail.com',
    description='PX4 uXRCE-DDS bridge exposing a stable flight API (Phase 1).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flight_demo = px4_bridge.flight_demo_node:main',
            'px4_bridge_node = px4_bridge.px4_bridge_node:main',
            'teleop = px4_bridge.teleop_node:main',
        ],
    },
)
