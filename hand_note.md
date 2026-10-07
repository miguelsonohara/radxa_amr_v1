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

## Ros2-jazzy navigation:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False

# Micro-ros
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

# Print tf tree
ros2 run tf2_tools view_frames

# Control 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

## Robot Bringup:
sudo systemctl enable --now robot_bringup.service
sudo systemctl restart robot_bringup.service
sudo systemctl stop robot_bringup.service
systemctl status robot_bringup.service
journalctl -u robot_bringup.service -f

## Video Kiosk:
sudo systemctl enable --now play_video.service
sudo systemctl restart play_video.service
sudo systemctl stop play_video.service
systemctl status play_video.service
journalctl -u play_video.service -f

## Ưu tiên CPU (chống robot cà giật)
# Kiosk bị ghim vào 4 core nhỏ (0-3) + nice 10; bringup chạy nice -5 và được
# scheduler đẩy lên core lớn (4-7). Kiểm tra lại khi có nghi vấn:
grep Cpus_allowed_list /proc/$(pgrep -x mpv)/status     # phải là 0-3
ps -o pid,ni,psr,comm -p $(pgrep -f micro_ros_agent)    # nice phải là -5

## Kiểm tra kiosk có dùng giải mã cứng Venus không (bắt buộc, đừng để về CPU)
# playback-time phải tăng dần và core-idle phải là False.
echo '{"command":["get_property","hwdec-current"]}' | python3 -c "
import json,socket,sys
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.connect('/tmp/mpvsocket')
s.sendall(sys.stdin.buffer.read());print(s.recv(4096).decode())"

# Kill all ROS nodes manually:
bash -c "pkill -9 -f bringup; pkill -9 -f ros2; pkill -9 -f nav2; pkill -9 -f slam_toolbox; pkill -9 -f pose_detector; pkill -9 -f usb_cam; pkill -9 -f sllidar; pkill -9 -f micro_ros; sleep 2; ps aux | grep -E 'ros|nav2|yolo'"