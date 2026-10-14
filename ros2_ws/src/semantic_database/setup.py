import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'semantic_database'

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
    description='SQLite semantic object store + QueryObjects service. Phase 6.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_database_node = semantic_database.semantic_database_node:main'
        ],
    },
)
