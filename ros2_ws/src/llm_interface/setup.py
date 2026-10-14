import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'llm_interface'

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
    description='LLM query interface grounded in the semantic DB (Ollama/cloud backends, tool calling). Phase 7.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'llm_node = llm_interface.llm_node:main',
            'llm_cli = llm_interface.llm_cli:main'
        ],
    },
)
