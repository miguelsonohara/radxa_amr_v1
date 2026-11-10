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

Cài đặt tất cả các package hệ thống cần thiết cho điều hướng, cảm biến, micro-ROS và thị giác máy tính:

```bash
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-camera-info-manager \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  libv4l-dev \
  v4l-utils \
  python3-opencv \
  python3-numpy \
  python3-serial
```

---

## 4. Chuẩn Bị & Cập Nhật Mã Nguồn Workspace

Khi clone repo chính `radxa_amr_v1` về thư mục người dùng:
```bash
cd ~
git clone https://github.com/miguelsonohara/radxa_amr_v1.git
cd ~/radxa_amr_v1
```

---

## 5. Biên Dịch Workspace

Quay về thư mục gốc của workspace và thực hiện build:

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

## 6. Cấu Hình Environment & Quyền Truy Cập Thiết Bị

### 6.1. Tự động source vào `~/.bashrc`
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/radxa_amr_v1/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6.2. Phân quyền truy cập Serial Port và Camera
Thêm user `rctrobot` vào group `dialout` và `video` để truy cập mạch ESP32, LiDAR và Camera mà không cần `sudo`:
```bash
sudo usermod -a -G dialout,video $USER
```
*(Cần log out và log in lại để phân quyền có hiệu lực)*.

---

## 7. Cài Đặt Dịch Vụ Tự Khởi Động (Systemd Services)

Chạy script cài đặt dịch vụ tự động cho Single-Board:
```bash
~/radxa_amr_v1/scripts/install_robot_services.sh
```
Script sẽ tự động đăng ký:
1. `robot_agent.service`: Daemon HTTP tại `http://127.0.0.1:5001` nhận lệnh điều khiển từ Web Gateway (`radxa_llm_web_v1`).
2. `robot_bringup.service`: Stack khởi động headless cho toàn bộ phần cứng và Nav2.

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
