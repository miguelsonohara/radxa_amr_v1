#!/usr/bin/env python3

"""
bringup.launch.py - Main launch file for the Receptionist Robot host stack.
Author: Senior Robotics Engineer

This launch file starts:
1. Micro-ROS Agent: Communicates with ESP32 (shares /odom, /tf, and receives /cmd_vel).
# 2. Lidar Node (sllidar_ros2): Pulls scan points from RPLidar and publishes to /scan.
# 2. Lidar Node (hclidar_driver_ros2): Pulls scan points from HCLiDAR and publishes to /scan.
3. Robot State Publisher: Parses Xacro URDF and publishes static transforms.
4. SLAM Toolbox: Builds map dynamically and publishes map->odom transform.
5. Nav2 Stack: Standard path planning and local control, utilizing custom costmaps.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Package Paths & Configuration File Locators
    # --------------------------------------------------------------------------
    description_dir = get_package_share_directory('receptionist_robot_description')
    bringup_dir = get_package_share_directory('receptionist_robot_bringup')
    
    # Locate our custom parameter files
    slam_params_path = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    # Locate navigation bringup launch file from nav2_bringup package
    nav2_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    
    # Locate SLAM Toolbox launch file from slam_toolbox package
    slam_launch_path = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    
    # Locate and process URDF Xacro file using the xacro library
    xacro_file = os.path.join(description_dir, 'urdf', 'receptionist_robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # --------------------------------------------------------------------------
    # 2. Launch Arguments (Declared for easy runtime customization)
    # --------------------------------------------------------------------------
    
    # Use Simulation Time: Set to 'true' if running inside Gazebo/Webots, 
    # but 'false' when running on the actual Radxa and ESP32 hardware.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (clock) time if true'
    )

    # Micro-ROS Serial Port: The serial interface connecting the Radxa to the ESP32.
    # Typically '/dev/ttyACM0' (USB OTG/direct) or '/dev/ttyUSB1' (UART converter).
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the micro-ROS Agent connection to ESP32'
    )

    # Micro-ROS Baudrate: Baud rate of the serial connection to the ESP32.
    # Must match the baudrate configured in the ESP32 micro-ROS firmware.
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate of the micro-ROS Agent connection (e.g. 115200 or 921600)'
    )

    # Lidar Serial Port: The interface where LiDAR is plugged in.
    # Usually '/dev/ttyUSB0'.
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the LiDAR sensor'
    )

    # Lidar Baudrate: Speed of communication with LiDAR.
    # # declare_lidar_baudrate = DeclareLaunchArgument(
    # #     'lidar_baudrate',
    # #     default_value='460800',
    # #     description='Baudrate for RPLidar (typically 115200 for A1/A2, 256000 for A3)'
    # # )
    declare_lidar_baudrate = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='Baudrate for HCLiDAR (typically 115200)'
    )

    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model',
        default_value='X2M',
        description='Model of the HCLiDAR sensor (e.g. X1, X2M)'
    )

    # --------------------------------------------------------------------------
    # 3. Node Declarations
    # --------------------------------------------------------------------------

    # A. Micro-ROS Agent Node
    # Runs the serial agent on the host to bridge low-level motor controllers on ESP32.
    # Communicates over serial using the defined port and baudrate.
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('serial_port'), '-b', LaunchConfiguration('serial_baudrate')],
        output='screen'
    )

    # B. Lidar Node (sllidar_ros2 vs hclidar_driver_ros2)
    # # lidar_node = Node(
    # #     package='sllidar_ros2',
    # #     executable='sllidar_node',
    # #     name='sllidar_node',
    # #     parameters=[{
    # #         'channel_type': 'serial',
    # #         'serial_port': LaunchConfiguration('lidar_port'),
    # #         'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
    # #         'frame_id': 'laser',  # Must match the link name in URDF
    # #         'inverted': False,    # True to mirror scans if mounted upside down
    # #         'angle_compensate': True
    # #     }],
    # #     output='screen'
    # # )
    lidar_node = Node(
        package='hclidar_driver_ros2',
        executable='hclidar_driver_ros2_node',
        name='hclidar_driver_ros2_node',
        parameters=[{
            'port': LaunchConfiguration('lidar_port'),
            'baudrate': LaunchConfiguration('lidar_baudrate'),
            'lidar_model': LaunchConfiguration('lidar_model'),
            'frame_id': 'laser',  # Must match the link name in URDF
        }],
        output='screen'
    )

    # C. Robot State Publisher Node
    # Publishes static TFs (e.g. base_link -> laser, base_link -> wheels) 
    # to the central `/tf` system, based on the processed Xacro XML.
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

    # --------------------------------------------------------------------------
    # 4. Included Launch Files (SLAM Toolbox & Nav2 Navigation)
    # --------------------------------------------------------------------------

    # D. SLAM Toolbox Node
    # Runs the online async mapping node. Generates `/map` and publishes map->odom TF.
    # Leverages our custom annotated parameters file.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # E. Nav2 Stack
    # Launches controller, planner, behavior server, smoother, collision monitor, 
    # and waypoint follower. It excludes localization (AMCL) since SLAM is active.
    # Uses our custom parameter file.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'
        }.items()
    )

    # --------------------------------------------------------------------------
    # 5. Assemble Launch Description
    # --------------------------------------------------------------------------
    ld = LaunchDescription()
    
    # Add Arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_serial_baudrate)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_lidar_baudrate)
    ld.add_action(declare_lidar_model)
    
    # Add Nodes
    ld.add_action(micro_ros_agent_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)
    
    # Add Included Launchers
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_navigation_launch)

    return ld
