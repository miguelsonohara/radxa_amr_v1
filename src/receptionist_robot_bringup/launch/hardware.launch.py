#!/usr/bin/env python3

"""
hardware.launch.py - Hardware drivers only for the Receptionist Robot.
Author: Senior Robotics Engineer

This launch file starts:
1. Micro-ROS Agent: Communicates with ESP32 (shares /odom, /tf, and receives /cmd_vel).
2. Lidar Node (sllidar_ros2): Pulls scan points from RPLidar and publishes to /scan.
3. Robot State Publisher: Parses Xacro URDF and publishes static transforms.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Package Paths & Configuration File Locators
    description_dir = get_package_share_directory('receptionist_robot_description')
    
    # Locate and process URDF Xacro file using the xacro library
    xacro_file = os.path.join(description_dir, 'urdf', 'receptionist_robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # 2. Launch Arguments (Declared for easy runtime customization)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (clock) time if true'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the micro-ROS Agent connection to ESP32'
    )

    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate of the micro-ROS Agent connection (e.g. 115200 or 921600)'
    )

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar sensor'
    )

    declare_lidar_baudrate = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='460800',
        description='Baudrate for RPLidar'
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
    
    ld.add_action(micro_ros_agent_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
