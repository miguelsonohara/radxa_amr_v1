#!/usr/bin/env python3

"""
hardware.launch.py - Hardware drivers only for the Receptionist Robot.
Author: Senior Robotics Engineer

This launch file starts:
1. Micro-ROS Agent: Communicates with ESP32 (shares /odom, /tf, and receives /cmd_vel).
# 2. Lidar Node (sllidar_ros2): Pulls scan points from RPLidar and publishes to /scan.
# 2. Lidar Node (hclidar_driver_ros2): Pulls scan points from HCLiDAR and publishes to /scan.
3. Robot State Publisher: Parses Xacro URDF and publishes static transforms.
"""

import glob
import os
import subprocess
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESP32_VIDS = ('1a86', '303a')   # QinHeng CH34x bridge / Espressif native USB
LIDAR_VIDS = ('10c4',)          # Silicon Labs CP210x (RPLidar / HCLiDAR)


def _usb_vid(dev):
    """Returns the lowercase USB vendor ID of a tty node ('' if unknown)."""
    try:
        out = subprocess.check_output(['udevadm', 'info', '-q', 'property', '-n', dev],
                                      text=True, stderr=subprocess.DEVNULL, timeout=2)
        for line in out.splitlines():
            if line.startswith('ID_VENDOR_ID='):
                return line.split('=', 1)[1].strip().lower()
    except Exception:
        pass
    return ''


def _usable(dev, exclude):
    """True if dev exists and is not the same physical node as exclude (symlinks resolved)."""
    if not os.path.exists(dev):
        return False
    return exclude is None or os.path.realpath(dev) != os.path.realpath(exclude)


def find_esp32_device(exclude=None):
    """Dynamically detects ESP32-S3 serial port across board swaps (never returns the LiDAR port)."""
    if _usable('/dev/robot_esp32', exclude):
        return '/dev/robot_esp32'
    # Anchor to usb-1a86_ prefix and verify VID so a LiDAR serial containing '1a86' in its hex UUID never matches
    for pat in ['/dev/serial/by-id/usb-1a86_*', '/dev/serial/by-id/usb-wch*', '/dev/serial/by-id/usb-Espressif_*', '/dev/serial/by-id/*Single_Serial*']:
        for dev in sorted(glob.glob(pat)):
            if _usable(dev, exclude) and _usb_vid(dev) not in LIDAR_VIDS:
                return dev
    candidates = sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
    for dev in candidates:
        if _usable(dev, exclude) and _usb_vid(dev) in ESP32_VIDS:
            return dev
    for dev in candidates:
        if _usable(dev, exclude) and _usb_vid(dev) not in LIDAR_VIDS:
            return dev
    return '/dev/ttyACM0'


def find_lidar_device(exclude=None):
    """Dynamically detects LiDAR serial port across sensor swaps (never returns the ESP32 port)."""
    for symlink in ['/dev/robot_lidar', '/dev/rplidar']:
        if _usable(symlink, exclude):
            return symlink
    for pat in ['/dev/serial/by-id/usb-Silicon_Labs_*', '/dev/serial/by-id/*CP210*', '/dev/serial/by-id/*RPLIDAR*', '/dev/serial/by-id/*rplidar*']:
        for dev in sorted(glob.glob(pat)):
            if _usable(dev, exclude) and _usb_vid(dev) not in ESP32_VIDS:
                return dev
    candidates = sorted(glob.glob('/dev/ttyUSB*'))
    for dev in candidates:
        if _usable(dev, exclude) and _usb_vid(dev) in LIDAR_VIDS:
            return dev
    for dev in candidates:
        if _usable(dev, exclude) and _usb_vid(dev) not in ESP32_VIDS:
            return dev
    return '/dev/ttyUSB0'


def generate_launch_description():
    # 1. Package Paths & Configuration File Locators
    description_dir = get_package_share_directory('receptionist_robot_description')
    
    # Locate and process URDF Xacro file using the xacro library
    xacro_file = os.path.join(description_dir, 'urdf', 'receptionist_robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    default_esp32_port = find_esp32_device()
    default_lidar_port = find_lidar_device(exclude=default_esp32_port)

    # 2. Launch Arguments (Declared for easy runtime customization)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (clock) time if true'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value=default_esp32_port,
        description='Serial port for the micro-ROS Agent connection to ESP32 (auto-detected)'
    )

    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='921600',
        description='Baudrate of the micro-ROS Agent connection (e.g. 115200 or 921600)'
    )

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value=default_lidar_port,
        description='Serial port for the LiDAR sensor (auto-detected)'
    )

    declare_lidar_baudrate = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='460800',
        description='Baudrate for RPLidar'
    )
    # declare_lidar_baudrate = DeclareLaunchArgument(
    #     'lidar_baudrate',
    #     default_value='115200',
    #     description='Baudrate for HCLiDAR'
    # )

    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model',
        default_value='X2M',
        description='Model of the HCLiDAR sensor (e.g. X1, X2M)'
    )

    # 3. Node Declarations
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('serial_port'), '-b', LaunchConfiguration('serial_baudrate')],
        output='screen'
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen'
    )

    # lidar_node = Node(
    #     package='hclidar_driver_ros2',
    #     executable='hclidar_driver_ros2_node',
    #     name='hclidar_driver_ros2_node',
    #     parameters=[{
    #         'port': LaunchConfiguration('lidar_port'),
    #         'baudrate': LaunchConfiguration('lidar_baudrate'),
    #         'lidar_model': LaunchConfiguration('lidar_model'),
    #         'frame_id': 'laser',
    #     }],
    #     output='screen'
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_xml
        }]
    )

    # 4. Assemble Launch Description
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_serial_baudrate)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_lidar_baudrate)
    ld.add_action(declare_lidar_model)
    
    ld.add_action(micro_ros_agent_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
