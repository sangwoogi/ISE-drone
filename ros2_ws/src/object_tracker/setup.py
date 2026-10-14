import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'object_tracker'

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
    description='2D track association (ByteTrack-style) on detections. Phase 6.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tracker_node = object_tracker.tracker_node:main'
        ],
    },
)
