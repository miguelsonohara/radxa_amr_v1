import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'receptionist_robot_behavior'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radxa',
    maintainer_email='radxa@todo.todo',
    description='Behavior Tree Orchestrator for Receptionist Robot AMR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'amr_bt_node = receptionist_robot_behavior.amr_bt_node:main',
        ],
    },
)
