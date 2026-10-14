import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'system_monitor'

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
    description='CPU/RAM/temperature/topic-rate metrics -> diagnostics + CSV. All phases.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'system_monitor_node = system_monitor.system_monitor_node:main'
        ],
    },
)
