# Hướng Dẫn Cài Đặt Môi Trường ROS 2 Lyrical Trên Radxa (Dragon Q6A)

Tài liệu này tổng hợp toàn bộ các bước thiết lập, cài đặt dependencies và build workspace sau khi clone repository `radxa_amr_v1` về một bo mạch Radxa mới.

---

## 1. Thiết Lập Locale & APT Repository

### 1.1. Cấu hình Locale (bắt buộc cho ROS 2)
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2. Thêm ROS 2 APT Repository (Ubuntu 26.04 Resolute / ROS 2 Lyrical)
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu resolute main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## 2. Cài Đặt ROS 2 Lyrical & Công Cụ Build

```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## 3. Cài Đặt Toàn Bộ Dependencies Cho Robot Stack

Cài đặt tất cả các package hệ thống cần thiết cho điều hướng, cảm biến, micro-ROS và thị giác máy tính:

```bash
sudo apt install -y \
  ros-lyrical-xacro \
  ros-lyrical-nav2-bringup \
  ros-lyrical-slam-toolbox \
  ros-lyrical-robot-localization \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-tf2-tools \
  ros-lyrical-micro-ros-msgs \
  ros-lyrical-camera-info-manager \
  ros-lyrical-rqt-image-view \
  ros-lyrical-cv-bridge \
  ros-lyrical-image-transport \
  libv4l-dev \
  v4l-utils \
  python3-opencv \
  python3-numpy
```

---

## 4. Chuẩn Bị & Cập Nhật Mã Nguồn Workspace

Khi clone repo chính `radxa_amr_v1` về thư mục người dùng:
```bash
cd ~
git clone https://github.com/miguelsonohara/radxa_amr_v1.git
cd ~/radxa_amr_v1/src
```

### 4.1. Clone các Driver bổ sung (nếu chưa có trong `src/`):

1. **Micro-ROS Agent (nhánh `lyrical`):**
   ```bash
   git clone -b lyrical https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent
   ```

2. **USB Camera Driver:**
   ```bash
   git clone https://github.com/ros-drivers/usb_cam.git usb_cam
   ```

3. **LiDAR Drivers (nếu chưa có):**
   ```bash
   git clone https://github.com/Slamtec/sllidar_ros2.git sllidar_ros2
   git clone https://github.com/manhbt/hclidar_driver_ros2.git hclidar_driver_ros2
   ```

---

## 5. Các Bản Vá (Patch) Bắt Buộc Cho ROS 2 Lyrical

Do ROS 2 Lyrical sử dụng GCC 15 và Modern CMake (loại bỏ macro cũ), các file sau cần được đảm bảo đã patch:

### 5.1. Patch `sllidar_ros2` & `hclidar_driver_ros2` (CMakeLists.txt)
> *Lưu ý: ROS 2 Lyrical đã xóa bỏ hàm `ament_target_dependencies`.*
* Trong `src/sllidar_ros2/CMakeLists.txt` và `src/hclidar_driver_ros2/CMakeLists.txt`:
  Thay thế toàn bộ các lời gọi `ament_target_dependencies(...)` bằng `target_link_libraries(...)` liên kết trực tiếp với `rclcpp::rclcpp`, `${sensor_msgs_TARGETS}`, `${std_srvs_TARGETS}`,... *(Trong repo này đã được chỉnh sửa sẵn)*.

### 5.2. Patch `usb_cam` (CMakeLists.txt)
* Trong file `src/usb_cam/CMakeLists.txt` (dòng 10):
  Thay:
  ```cmake
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  ```
  Thành:
  ```cmake
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error=deprecated-declarations)
  ```
  *(Để tránh GCC 15 dừng build do cảnh báo deprecation API của camera)*.

---

## 6. Biên Dịch Workspace

Quay về thư mục gốc của workspace và thực hiện build:

```bash
cd ~/radxa_amr_v1
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

Sau khi build thành công:
```bash
source install/setup.bash
```

---

## 7. Cấu Hình Environment & Quyền Truy Cập Thiết Bị

### 7.1. Tự động source vào `~/.bashrc`
```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
echo "source ~/radxa_amr_v1/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7.2. Phân quyền truy cập Serial Port và Camera
Thêm user hiện tại vào group `dialout` và `video` để truy cập mạch ESP32, LiDAR và Camera mà không cần `sudo`:
```bash
sudo usermod -a -G dialout,video $USER
```
*(Cần log out và log in lại để phân quyền có hiệu lực)*.

---

## 8. Kiểm Tra Hoạt Động (Sanity Check)

### 8.1. Kiểm tra không cần cắm phần cứng (Giả lập để xem trên RViz & rqt từ PC)
```bash
python3 ~/radxa_amr_v1/scripts/publish_test_data.py
```
* Trên máy PC (Jazzy/Lyrical): Mở `rqt_image_view` (chọn topic `/image_raw`) và `rviz2` (Fixed frame: `base_link`, add `LaserScan` topic `/scan`).

### 8.2. Chạy toàn bộ Robot Bringup (khi đã cắm phần cứng)
```bash
ros2 launch receptionist_robot_bringup bringup.launch.py \
  serial_port:=/dev/ttyACM0 \
  lidar_port:=/dev/ttyUSB0
```
