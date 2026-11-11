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

import glob
import os
import subprocess
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # --------------------------------------------------------------------------
    # 1. Package Paths & Configuration File Locators
    # --------------------------------------------------------------------------
    description_dir = get_package_share_directory('receptionist_robot_description')
    bringup_dir = get_package_share_directory('receptionist_robot_bringup')
    
    # Locate our custom parameter files
    slam_params_path = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    ekf_params_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    
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
    default_esp32_port = find_esp32_device()
    default_lidar_port = find_lidar_device(exclude=default_esp32_port)
    default_cam_port = '/dev/my_camera' if os.path.exists('/dev/my_camera') else ('/dev/video0' if os.path.exists('/dev/video0') else '/dev/video1')
    
    # Use Simulation Time: Set to 'true' if running inside Gazebo/Webots, 
    # but 'false' when running on the actual Radxa and ESP32 hardware.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (clock) time if true'
    )

    # Micro-ROS Serial Port: The serial interface connecting the Radxa to the ESP32.
    # Typically '/dev/ttyACM0' (USB OTG/direct) or '/dev/ttyUSB0' (UART converter).
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value=default_esp32_port,
        description='Serial port for the micro-ROS Agent connection to ESP32 (auto-detected)'
    )

    # Micro-ROS Baudrate: Baud rate of the serial connection to the ESP32.
    # Must match the baudrate configured in the ESP32 micro-ROS firmware.
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='921600',
        description='Baudrate of the micro-ROS Agent connection (e.g. 115200 or 921600)'
    )

    # Lidar Serial Port: The interface where LiDAR is plugged in.
    # Usually '/dev/ttyUSB0'.
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value=default_lidar_port,
        description='Serial port for the LiDAR sensor (auto-detected)'
    )

    # Lidar Baudrate: Speed of communication with LiDAR (460800 for RPLidar C1).
    declare_lidar_baudrate = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='460800',
        description='Baudrate for RPLidar C1 (460800)'
    )

    declare_scan_mode = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode for RPLidar C1 (Standard or Dense)'
    )

    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model',
        default_value='X2M',
        description='Model of the HCLiDAR sensor (e.g. X1, X2M)'
    )

    declare_camera_device = DeclareLaunchArgument(
        'camera_device',
        default_value=default_cam_port,
        description='Video device path for usb_cam (auto-detected)'
    )

    declare_enable_keepout = DeclareLaunchArgument(
        'enable_keepout',
        default_value='false',
        description='Enable Nav2 keepout costmap filters (default: false)'
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

    # B. Lidar Node (sllidar_ros2 configured for RPLidar C1)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',  # Must match the link name in URDF
            'inverted': False,    # True to mirror scans if mounted upside down
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode'),
            'range_min': 0.25,   # <--- THÊM DÒNG NÀY: Lidar tự bỏ qua các điểm < 0.25m
            'range_max': 12.0
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
    #         'frame_id': 'laser',  # Must match the link name in URDF
    #     }],
    #     output='screen'
    # )

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
    # D. Robot Localization Node (EKF Filter)
    # Fuses raw wheel odometry (/odom) and IMU (/imu) into /odom/filtered, publishing odom -> base_link TF.
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )

    # G. USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'raw_mjpeg',
            'io_method': 'mmap'
        }],
        output='screen'
    )

    # H. YOLOv11 Pose Detector Node
    yolo_pose_node = Node(
        package='yolov11_pose_detector',
        executable='pose_detector_node',
        name='yolov11_pose_detector',
        output='screen'
    )

    # I. Nav2 Keepout Filter Nodes (Restricted Stair/Hole Zones - Set false by default)
    ws_dir = os.path.expanduser('~/radxa_amr_v1')
    mask_yaml_file = os.path.join(ws_dir, 'map', 'keepout_mask.yaml')
    
    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_keepout')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': mask_yaml_file,
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map'
        }]
    )

    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_keepout')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base_service_name': '/costmap_filter_info'
        }]
    )

    costmap_filter_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_keepout')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }]
    )

    # J. Wi-Fi Credentials Receiver Node
    wifi_receiver_node = Node(
        package='wifi_receiver',
        executable='wifi_receiver_node',
        name='wifi_receiver_node',
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 4. Included Launch Files (SLAM Toolbox & Nav2 Navigation)
    # --------------------------------------------------------------------------

    # E. SLAM Toolbox Node
    # Runs the online async mapping node. Generates `/map` and publishes map->odom TF.
    # Leverages our custom annotated parameters file.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'params_file': slam_params_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # F. Nav2 Stack
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
    ld.add_action(declare_scan_mode)
    ld.add_action(declare_lidar_model)
    ld.add_action(declare_camera_device)
    ld.add_action(declare_enable_keepout)
    
    # Add Core Driver & Hardware Nodes Immediately
    ld.add_action(micro_ros_agent_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_localization_node)
    ld.add_action(usb_cam_node)
    ld.add_action(filter_mask_server_node)
    ld.add_action(costmap_filter_info_server_node)
    ld.add_action(costmap_filter_lifecycle_manager_node)
    ld.add_action(wifi_receiver_node)
    ld.add_action(slam_toolbox_launch)
    
    # Delayed Startup (4 seconds) for Nav2 & Perception to allow TF trees to stabilize
    delayed_nav2_and_perception = TimerAction(
        period=4.0,
        actions=[
            yolo_pose_node,
            nav2_navigation_launch
        ]
    )
    ld.add_action(delayed_nav2_and_perception)

    return ld

