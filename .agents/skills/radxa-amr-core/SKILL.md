---
name: radxa-amr-core
description: >-
  Operational workflows, build procedures, hardware bringup, ROS 2 navigation, LiDAR SLAM,
  ESP32 micro-ROS interfacing, and REST daemon control runbook for the Receptionist Robot Motion
  Core workspace (radxa_amr_v1) running on the Banana Pi CM4 IO (BPI-CM4-IO / bananapicm4io) single-board computer.
  Use this skill whenever building, launching, configuring, testing, or debugging the robot navigation,
  SLAM mapping, sensors, micro-ROS motor controller, or robot core daemon.
---

# Receptionist Robot Motion Core Runbook (`radxa_amr_v1`)

This skill documents actionable operational procedures for developing, building, launching, and debugging the robot motion, SLAM, Nav2, sensor bringup, and REST management daemon in `radxa_amr_v1`.

---

## ⚡ QUY TẮC THỰC THI LỆNH CHO AI AGENT (RUNNER PROTOCOL: SINGLE-APPROVAL)

> [!IMPORTANT]
> **Quy tắc bắt buộc khi Agent cần chạy lệnh bash, build hoặc kiểm thử Python:**
> 1. **Dùng file runner có tên cố định (`scripts/runner.sh` hoặc `scripts/runner.py`):**
>    - Khi cần chạy chuỗi lệnh bash, build hay chạy code kiểm tra: **CHỈ SỬ DỤNG MỘT FILE CỐ ĐỊNH** là `scripts/runner.sh` (hoặc `scripts/runner.py` cho script Python).
>    - **TUYỆT ĐỐI KHÔNG** tạo file mới rồi xóa đi mỗi lần chạy (`không xóa tạo mỗi lần`). Hãy tái sử dụng và cập nhật / ghi đè nội dung file runner cố định này qua `write_file` (Overwrite=true) hoặc `edit_file`.
> 2. **Chấp thuận 1 lần duy nhất (Single Approval):**
>    - **KHÔNG** gọi nhiều lệnh `run_command` rời rạc liên tiếp bắt người dùng phải bấm xác nhận (Accept) nhiều lần.
>    - Toàn bộ chuỗi lệnh cần chạy hoặc code kiểm thử phải được viết sẵn vào file runner cố định.
>    - Sau đó chỉ gọi `run_command` chạy file runner đó **đúng 1 lần duy nhất** (ví dụ: `bash /home/rctrobot/radxa_amr_v1/scripts/runner.sh`).
> 3. **Lợi ích:** Người dùng chỉ cần xem qua diff của file runner quen thuộc và bấm **Accept 1 lần** là toàn bộ chuỗi tác vụ / kiểm thử được thực thi xong, tránh làm rác thư mục dự án và tiết kiệm tối đa thời gian.

---

## 1. Build and Environment Setup

### 1.1. Build the Workspace
Always build with `--symlink-install` so Python scripts, configurations, and launch file changes take effect without recompilation:
```bash
cd ~/radxa_amr_v1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 1.2. Source the Workspace
```bash
source /opt/ros/jazzy/setup.bash
source ~/radxa_amr_v1/install/setup.bash
```

### 1.3. Environment Variables (Single-Board Architecture)
On the BPI-CM4-IO single-board setup, all inter-workspace communication runs via **Shared Memory (SHM)** and loopback:
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset ROS_STATIC_PEERS
```

---

## 2. Managing Systemd Background Services

The robot employs two systemd background services managed via `scripts/install_robot_services.sh`:

### 2.1. One-Time Service Installation
```bash
~/radxa_amr_v1/scripts/install_robot_services.sh
```

### 2.2. Service A: Robot Core Agent Daemon (`robot_agent.service`)
Listens on `http://127.0.0.1:5001` (and `0.0.0.0:5001`) to serve mode switching and status requests from the Web Gateway:
```bash
# Start / Stop / Restart
sudo systemctl enable --now robot_agent.service
sudo systemctl restart robot_agent.service
sudo systemctl status robot_agent.service

# View live daemon logs
journalctl -u robot_agent.service -f
```

### 2.3. Service B: Robot Bringup Stack (`robot_bringup.service`)
Headless service that manages the full robot bringup (micro-ROS, LiDAR, Nav2, SLAM, Camera, YOLO):
```bash
# Start / Stop / Restart
sudo systemctl start robot_bringup.service
sudo systemctl stop robot_bringup.service
sudo systemctl restart robot_bringup.service
sudo systemctl status robot_bringup.service

# View bringup logs
journalctl -u robot_bringup.service -f
```

---

## 3. Hardware Interfaces & Device Detection

### 3.1. Verify Connected Peripherals
```bash
# Check ESP32 motor board & LiDAR USB serial ports
ls -la /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Check USB Camera
ls -la /dev/video* 2>/dev/null
```

### 3.2. Serial Permissions
Ensure user `rctrobot` has dialout and video permissions:
```bash
sudo usermod -a -G dialout,video $USER
```

### 3.3. Device Port Resolution in `start_robot_bringup.sh`
* **ESP32-S3:** Resolves `/dev/serial/by-id/usb-1a86_USB_Single_Serial*` or fallback `/dev/ttyACM0` (Baudrate: `921600`).
* **LiDAR:** Resolves `/dev/serial/by-id/usb-Silicon_Labs*` or fallback `/dev/ttyUSB0` (Baudrate: `115200` for HCLiDAR / `460800` for RPLidar).
* **Camera:** Resolves `/dev/my_camera` or fallback `/dev/video0`.

---

## 4. REST Control Daemon API (`127.0.0.1:5001`)

The daemon at [scripts/robot_agent.py](file:///home/rctrobot/radxa_amr_v1/scripts/robot_agent.py) accepts HTTP requests:

### 4.1. Check Status
```bash
curl -s http://127.0.0.1:5001/api/robot/status | jq .
```
Response:
```json
{
  "mode": "localization",
  "home_pose": [0.0, 0.0, 0.0],
  "current_map": "/home/rctrobot/radxa_amr_v1/map/demo_map1",
  "current_map_name": "demo_map1",
  "last_saved_map": "demo_map1",
  "slam_active": true,
  "bringup_service_active": true
}
```

### 4.2. Scan Saved Maps
```bash
curl -s http://127.0.0.1:5001/api/robot/maps | jq .
```

### 4.3. Switch Mode (SLAM Mapping <-> Nav2 Localization)
* **Switch to SLAM Mapping:**
  ```bash
  curl -X POST http://127.0.0.1:5001/api/robot/mode \
    -H "Content-Type: application/json" \
    -d '{"mode": "mapping"}'
  ```
* **Switch to Nav2 Localization with Map:**
  ```bash
  curl -X POST http://127.0.0.1:5001/api/robot/mode \
    -H "Content-Type: application/json" \
    -d '{"mode": "localization", "map_name": "demo_map1"}'
  ```

### 4.4. Save Map Dual-Format (PoseGraph + 2D Yaml/Pgm)
```bash
curl -X POST http://127.0.0.1:5001/api/robot/save-map \
  -H "Content-Type: application/json" \
  -d '{"map_name": "office_floor1"}'
```

### 4.5. Manage Home Position
* **Record current robot pose as Home:**
  ```bash
  curl -X POST http://127.0.0.1:5001/api/robot/home \
    -H "Content-Type: application/json" \
    -d '{"action": "set_home", "home_pose": [1.5, 2.0, 0.0]}'
  ```
* **Navigate back to Home:**
  ```bash
  curl -X POST http://127.0.0.1:5001/api/robot/home \
    -H "Content-Type: application/json" \
    -d '{"action": "go_home"}'
  ```

---

## 5. Manual Launch & Teleoperation Debugging

### 5.1. Launch Robot Bringup Manually (without service)
```bash
source /opt/ros/jazzy/setup.bash && source ~/radxa_amr_v1/install/setup.bash
ros2 launch receptionist_robot_bringup bringup.launch.py \
  serial_port:=/dev/ttyACM0 \
  lidar_port:=/dev/ttyUSB0 \
  camera_device:=/dev/video0 \
  use_sim_time:=false
```

### 5.2. Teleoperate with Keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

### 5.3. Monitor Core Topics
```bash
# Odometry from ESP32 wheel encoders
ros2 topic echo /odom --once

# LiDAR laser scans
ros2 topic echo /scan --once

# Nav2 or teleop velocity output
ros2 topic echo /cmd_vel --once

# TF Tree inspection
ros2 run tf2_tools view_frames
```

### 5.4. Emergency Node Termination
Kill all ROS 2 nodes, Nav2 servers, and bringup scripts:
```bash
bash -c "pkill -9 -f bringup; pkill -9 -f ros2; pkill -9 -f nav2; pkill -9 -f slam_toolbox; pkill -9 -f pose_detector; pkill -9 -f usb_cam; pkill -9 -f sllidar; pkill -9 -f hclidar; pkill -9 -f micro_ros; pkill -9 -f robot_agent; sleep 1; ps aux | grep -E 'ros|nav2|yolo'"
```
