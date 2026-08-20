from setuptools import find_packages, setup

package_name = 'wifi_receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radxa',
    maintainer_email='radxa@todo.todo',
    description='ROS 2 Wi-Fi Credentials Receiver Daemon for Radxa 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_receiver_node = wifi_receiver.wifi_receiver_node:main',
        ],
    },
)
