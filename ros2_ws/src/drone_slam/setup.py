import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_slam'

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
    description='RTAB-Map bringup, map->odom TF, trajectory evaluation (ATE/RPE). Phase 4.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_eval = drone_slam.slam_eval:main',
            'gt_pose_node = drone_slam.gt_pose_node:main',
            'trajectory_recorder = drone_slam.trajectory_recorder:main'
        ],
    },
)
