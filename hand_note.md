# Build command: 
colcon build --symlink-install && source install/setup.sh

# LiDAR:
# ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=460800
ros2 launch hclidar_driver_ros2 hclidar_launch.py
# Or run node directly: ros2 run hclidar_driver_ros2 hclidar_driver_ros2_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p frame_id:=laser

# Base Link/Laser:
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser 

# Base Footprint:
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id base_footprint

# SLAM Mapping:
ros2 run cartographer_ros cartographer_node     -configuration_directory ~/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/config       -configuration_basename localization.lua     -load_state_filename ~/turtlebot3_ws/map/test_map.pbstream     --ros-args -p use_sim_time:=False

# SLAM Map
ros2 run cartographer_ros cartographer_occupancy_grid_node     --ros-args -p resolution:=0.05     -p use_sim_time:=False     -p publish_period_sec:=1.0

# Run bringup
ros2 launch turtlebot3_bringup robot.launch.py

# Run navigation:
## Turtlebot3 navigation: 
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/radxa/turtlebot3_ws/map/test_map.pbstream use_sim_time:=False

## Ros2-jazzy navigation:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False

# Micro-ros
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

# IN DEV REMOTE PC: 
ros2 launch nav2_bringup rviz_launch.py

# Print tf tree
ros2 run tf2_tools view_frames

# Control 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Service
sudo systemctl daemon-reload && sudo systemctl enable --now robot_bringup.service

bash -c "pkill -9 -f bringup; pkill -9 -f ros2; pkill -9 -f nav2; pkill -9 -f slam_toolbox; pkill -9 -f pose_detector; pkill -9 -f usb_cam; pkill -9 -f sllidar; pkill -9 -f micro_ros; sleep 2; ps aux | grep -E 'ros|nav2|yolo'"

# Video Kiosk (GDM3 / X11 Session + MPV):
## Service quản lý: gdm3.service (chạy session receptionist-kiosk.desktop -> kiosk-session.sh)

## Restart video (kiosk-session sẽ tự bật lại ngay lập tức):
pkill -f "/usr/bin/mpv"

## Stop video & kiosk loop hoàn toàn:
pkill -f "kiosk-session.sh"; pkill -f "/usr/bin/mpv"

## Start lại video kiosk thủ công:
DISPLAY=:0 /home/radxa/receptionist_robot_ws/play_video_kiosk.sh &
# Hoặc restart toàn bộ session GDM: sudo systemctl restart gdm3

## Kiểm tra trạng thái video đang phát:
python3 -c 'import socket, json; s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect("/tmp/mpvsocket"); s.sendall(json.dumps({"command": ["get_property", "time-pos"]}).encode()+b"\n"); print("Time:", s.recv(1024).decode().strip()); s.close()'

## Pause / Resume video qua IPC socket:
python3 -c 'import socket, json; s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect("/tmp/mpvsocket"); s.sendall(json.dumps({"command": ["cycle", "pause"]}).encode()+b"\n"); print(s.recv(1024).decode().strip()); s.close()'

## Đổi video động không chớp màn hình:
python3 -c 'import socket, json, sys; video=sys.argv[1] if len(sys.argv)>1 else "/home/radxa/receptionist_robot_ws/xoaylai180.mp4"; s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect("/tmp/mpvsocket"); s.sendall(json.dumps({"command": ["loadfile", video]}).encode()+b"\n"); print(s.recv(1024).decode().strip()); s.close()' /home/radxa/receptionist_robot_ws/xoaylai180.mp4