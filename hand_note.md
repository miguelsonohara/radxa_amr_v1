# Hướng Dẫn Vận Hành Single-Board BPI-CM4-IO (`radxa_amr_v1`)

### 1. Build workspace:
```bash
cd ~/radxa_amr_v1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

---

### 2. Quản lý Dịch Vụ Hệ Thống (Systemd Services)

Cài đặt 2 dịch vụ hệ thống chạy tự động khi boot:
```bash
~/radxa_amr_v1/scripts/install_robot_services.sh
```

#### A. Robot Agent Daemon (Cổng 5001 - Nhận lệnh từ Web Gateway):
```bash
sudo systemctl enable --now robot_agent.service
sudo systemctl restart robot_agent.service
sudo systemctl status robot_agent.service
journalctl -u robot_agent.service -f
```

#### B. Robot Bringup (Nav2, SLAM, micro-ROS, LiDAR, Camera, YOLO):
```bash
sudo systemctl enable --now robot_bringup.service
sudo systemctl restart robot_bringup.service
sudo systemctl stop robot_bringup.service
sudo systemctl status robot_bringup.service
journalctl -u robot_bringup.service -f
```

---

### 3. Kiểm Tra Kết Nối Thiết Bị Phần Cứng
```bash
# Kiểm tra ESP32 (micro-ROS) & LiDAR
ls -la /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Kiểm tra Camera USB
ls -la /dev/video* 2>/dev/null

# Khởi chạy thủ công Robot Bringup:
~/radxa_amr_v1/start_robot_bringup.sh
```

---

### 4. Lệnh ROS 2 Debug Nhanh
```bash
# Source môi trường
source /opt/ros/jazzy/setup.bash && source ~/radxa_amr_v1/install/setup.bash

# LiDAR:
ros2 launch hclidar_driver_ros2 hclidar_launch.py

# Micro-ROS Agent thủ công:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600

# Điều khiển Teleop bàn phím:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Kiểm tra TF Tree & Topic:
ros2 run tf2_tools view_frames
ros2 topic echo /odom --once
ros2 topic echo /scan --once

# Dừng tất cả tiến trình ROS thủ công:
bash -c "pkill -9 -f bringup; pkill -9 -f ros2; pkill -9 -f nav2; pkill -9 -f slam_toolbox; pkill -9 -f pose_detector; pkill -9 -f usb_cam; pkill -9 -f sllidar; pkill -9 -f hclidar; pkill -9 -f micro_ros; pkill -9 -f robot_agent; sleep 1; ps aux | grep -E 'ros|nav2|yolo'"
```
