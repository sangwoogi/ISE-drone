import os
from glob import glob

from setuptools import setup

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ISE-drone',
    maintainer_email='drknu12@gmail.com',
    description='Bringup launch files for the ISE-drone simulation stack.',
    license='MIT',
)
