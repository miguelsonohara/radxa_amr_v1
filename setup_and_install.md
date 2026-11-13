# Hướng Dẫn Cài Đặt Môi Trường ROS 2 Jazzy Trên BPI-CM4-IO (bananapicm4io)

Tài liệu này tổng hợp toàn bộ các bước thiết lập, cài đặt dependencies và build workspace sau khi clone repository `radxa_amr_v1` về bo mạch Banana Pi BPI-CM4-IO (Armbian 24.04 Noble / ROS 2 Jazzy Jalisco).

---

## 1. Thiết Lập Locale & APT Repository

### 1.1. Cấu hình Locale (bắt buộc cho ROS 2)
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2. Thêm ROS 2 APT Repository (Ubuntu 24.04 Noble / ROS 2 Jazzy)
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## 2. Cài Đặt ROS 2 Jazzy & Công Cụ Build

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## 3. Cài Đặt Toàn Bộ Dependencies Cho Robot Stack

Cài đặt tất cả các package hệ thống cần thiết cho điều hướng (Nav2), SLAM, EKF, camera (`usb_cam` cần FFmpeg + OpenCV dev headers), micro-ROS và thị giác máy tính (ONNX Runtime CPU):

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-camera-info-manager \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  libopencv-dev \
  libavcodec-dev \
  libavutil-dev \
  libswscale-dev \
  libv4l-dev \
  v4l-utils \
  python3-opencv \
  python3-numpy \
  python3-serial \
  python3-pydantic \
  python3-pip

pip3 install onnxruntime --break-system-packages
```

---

## 4. Chuẩn Bị & Cập Nhật Mã Nguồn Workspace

### 4.1. Clone repository chính
```bash
cd ~
git clone https://github.com/miguelsonohara/radxa_amr_v1.git
cd ~/radxa_amr_v1
```

### 4.2. Chuẩn bị các package phụ thuộc trong `src/` (LiDAR, Camera, micro-ROS)

Các package phần cứng gồm **RPLidar (`sllidar_ros2`)**, **HCLiDAR (`hclidar_driver_ros2`)**, **USB Camera (`usb_cam`)**, **`micro-ROS-Agent`** và **`micro_ros_msgs`** được duy trì từ các repository chính thức. Khi clone mới workspace, hãy chạy khối lệnh sau để đảm bảo toàn bộ thư viện được tải đầy đủ và đúng nhánh:

```bash
cd ~/radxa_amr_v1

# 1. RPLidar Driver (Slamtec sllidar_ros2)
if [ ! -d "src/sllidar_ros2" ]; then
  git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2
fi

# 2. HCLiDAR Driver (X2M / Camsense)
if [ ! -d "src/hclidar_driver_ros2" ]; then
  git clone -b humble https://github.com/manhbt/hclidar_driver_ros2.git src/hclidar_driver_ros2
fi

# 3. USB Camera Driver (V4L2)
if [ ! -d "src/usb_cam" ]; then
  git clone https://github.com/ros-drivers/usb_cam.git src/usb_cam
fi

# 4. micro_ros_msgs (Bắt buộc nhánh jazzy)
if [ ! -d "src/micro_ros_msgs" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
fi

# 5. micro-ROS-Agent (Bắt buộc nhánh jazzy)
# Lưu ý: Nhánh mặc định của micro-ROS-Agent là kilted (yêu cầu Fast-DDS v3, gây lỗi trên ROS 2 Jazzy).
if [ ! -d "src/micro-ROS-Agent" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
else
  git -C src/micro-ROS-Agent checkout jazzy
fi
```

---

## 5. Biên Dịch Workspace

Quay về thư mục gốc của workspace và thực hiện build (lần build đầu tiên của `micro_ros_agent` sẽ tải `xrceagent` v2.4.3 và biên dịch trong khoảng 4–5 phút):

```bash
cd ~/radxa_amr_v1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Sau khi build thành công:
```bash
source install/setup.bash
```

---

## 6. Cấu Hình Environment, Udev Rules & Quyền Truy Cập Thiết Bị

### 6.1. Tự động source vào `~/.bashrc`
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/radxa_amr_v1/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6.2. Cài đặt Udev Rules & Phân quyền Serial / Camera
Chạy script tạo symlink cố định (`/dev/robot_esp32`, `/dev/robot_lidar`, `/dev/rplidar`) và tự động thêm user `rctrobot` vào group `dialout`, `video`:
```bash
~/radxa_amr_v1/scripts/create_robot_udev_rules.sh
```
*(Lưu ý: Ngay cả khi chưa chạy udev rules hoặc khi thay thế bo ESP32 / LiDAR khác mã Serial ID, `start_robot_bringup.sh` và `bringup.launch.py` vẫn tự động quét theo USB Vendor ID `1a86/303a` và `10c4`).*

---

## 7. Cài Đặt Dịch Vụ Tự Khởi Động (Systemd Services)

Chạy script cài đặt dịch vụ tự động cho Single-Board:
```bash
~/radxa_amr_v1/scripts/install_robot_services.sh
```
Script sẽ tự động thực hiện:
1. Đăng ký và chạy `robot_agent.service`: Daemon HTTP tại `http://127.0.0.1:5001` nhận lệnh điều khiển từ Web Gateway (`radxa_llm_web_v1`).
2. Đăng ký `robot_bringup.service`: Stack khởi động headless cho toàn bộ phần cứng, SLAM, Nav2 và YOLO.
3. Tạo quyền `NOPASSWD` trong `/etc/sudoers.d/rctrobot-robot-services` để `robot_agent.service` có thể `start/stop/restart` dịch vụ `robot_bringup.service` mà không bị treo chờ mật khẩu.

---

## 8. Kiểm Tra Hoạt Động (Sanity Check)

### 8.1. Kiểm tra REST Daemon:
```bash
curl http://127.0.0.1:5001/api/robot/status
curl http://127.0.0.1:5001/api/robot/maps
```

### 8.2. Chạy toàn bộ Robot Bringup thủ công:
```bash
~/radxa_amr_v1/start_robot_bringup.sh
```
hoặc qua service:
```bash
sudo systemctl start robot_bringup.service
journalctl -u robot_bringup.service -f
```
