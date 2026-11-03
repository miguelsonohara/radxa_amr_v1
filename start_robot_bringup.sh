#!/bin/bash
# Headless ROS 2 bringup for plug-and-play boot.
# Starts Nav2, SLAM, LiDAR, ESP32 (micro-ROS), USB cam, YOLO pose.
# Does not start RViz — visualize from a remote PC.
# Do not use `set -u`: /opt/ros/jazzy/setup.bash references unset AMENT_* vars.

export HOME=/home/radxa
export USER=radxa
cd "$HOME/receptionist_robot_ws"

# Match interactive shell ROS networking (remote RViz on 10.254.254.1)
source /opt/ros/jazzy/setup.bash
source /home/radxa/receptionist_robot_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export ROS_STATIC_PEERS="10.254.254.1;192.168.16.28;192.168.16.27"

LIDAR_DEV="/dev/ttyHS3"
CAM_DEV="/dev/my_camera"

find_esp32_dev() {
    # 1. Check udev persistent symlink
    if [ -e "/dev/esp32_serial" ]; then
        echo "/dev/esp32_serial"
        return 0
    fi
    # 2. Check any 1a86 USB serial by-id pattern (CH340/CH341/CH343/CH347)
    for dev in /dev/serial/by-id/usb-1a86_*; do
        if [ -e "$dev" ]; then
            echo "$dev"
            return 0
        fi
    done
    # 3. Check any Espressif USB serial by-id pattern
    for dev in /dev/serial/by-id/usb-Espressif_*; do
        if [ -e "$dev" ]; then
            echo "$dev"
            return 0
        fi
    done
    # 4. Standard serial fallbacks
    if [ -e "/dev/ttyACM0" ]; then
        echo "/dev/ttyACM0"
        return 0
    fi
    if [ -e "/dev/ttyUSB0" ]; then
        echo "/dev/ttyUSB0"
        return 0
    fi
    return 1
}

wait_for_esp32_dev() {
    local timeout_s="${1:-90}"
    local elapsed=0
    echo "[bringup] waiting for ESP32 (micro-ROS)..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        local dev
        dev=$(find_esp32_dev)
        if [ -n "$dev" ] && [ -e "$dev" ]; then
            echo "[bringup] ESP32 (micro-ROS) ready: $dev"
            ESP32_DEV="$dev"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[bringup] WARNING: ESP32 not found after ${timeout_s}s — falling back to /dev/esp32_serial"
    ESP32_DEV="/dev/esp32_serial"
    return 1
}

wait_for_dev() {
    local path="$1"
    local label="$2"
    local timeout_s="${3:-90}"
    local elapsed=0
    echo "[bringup] waiting for ${label}: ${path}"
    while [ ! -e "$path" ] && [ "$elapsed" -lt "$timeout_s" ]; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
    if [ -e "$path" ]; then
        echo "[bringup] ${label} ready"
        return 0
    fi
    echo "[bringup] WARNING: ${label} not found after ${timeout_s}s — launching anyway"
    return 1
}

# Normal application reset sequence: DTR=False (GPIO0=HIGH), pulse RTS (EN=LOW then HIGH)
reset_esp32_hardware() {
    local dev="$1"
    if [ -e "$dev" ]; then
        echo "[bringup] pulsing RTS to perform clean hardware reset of ESP32..."
        python3 -c "
import serial, time
try:
    s = serial.Serial('$dev', 115200)
    s.dtr = False
    s.rts = True
    time.sleep(0.2)
    s.rts = False
    s.dtr = False
    s.close()
    print('[bringup] ESP32 normal run-mode reset complete.')
except Exception as e:
    print(f'[bringup] WARNING: ESP32 reset failed: {e}')
" 2>/dev/null || true
        sleep 0.5
    fi
}

# Ensure system clock is synchronized with NTP if network is available
# (Prevents a 14-hour clock jump after micro-ROS connects, which breaks EKF & TF)
echo "[bringup] waiting for system clock synchronization (NTP)..."
sync_elapsed=0
while [ "$sync_elapsed" -lt 20 ]; do
    if timedatectl 2>/dev/null | grep -q "System clock synchronized: yes"; then
        echo "[bringup] system clock synchronized: $(date)"
        break
    fi
    sleep 1
    sync_elapsed=$((sync_elapsed + 1))
done
if [ "$sync_elapsed" -ge 20 ]; then
    echo "[bringup] WARNING: NTP sync timed out after 20s. Proceeding with system time $(date)"
fi

wait_for_esp32_dev 90
wait_for_dev "$LIDAR_DEV" "LiDAR" 90
wait_for_dev "$CAM_DEV" "USB camera" 30

# Reset ESP32 hardware cleanly so it synchronizes clock with Linux host immediately
reset_esp32_hardware "$ESP32_DEV"

# Prevent Qualcomm UART GENI driver from power-saving autosuspend on LiDAR port
echo "on" > /sys/class/tty/ttyHS3/device/power/control 2>/dev/null || true

ros2 launch receptionist_robot_bringup bringup.launch.py \
    serial_port:="$ESP32_DEV" \
    lidar_port:="$LIDAR_DEV" \
    lidar_baudrate:=115200 \
    lidar_model:=X2A \
    camera_device:="$CAM_DEV" \
    use_sim_time:=false &
LAUNCH_PID=$!
trap 'echo "[bringup] shutting down..."; kill -TERM "$LAUNCH_PID" 2>/dev/null; wait "$LAUNCH_PID"' TERM INT

echo "[bringup] waiting for ESP32 /odom before confirming Nav2..."
elapsed=0
while [ "$elapsed" -lt 60 ]; do
    if timeout 8 ros2 topic echo /odom --once >/dev/null 2>&1; then
        echo "[bringup] /odom is publishing"
        break
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

echo "[bringup] ensuring Nav2 lifecycle is active"
elapsed=0
while [ "$elapsed" -lt 90 ]; do
    if timeout 6 ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active \[3\]"; then
        echo "[bringup] Nav2 is active"
        break
    fi
    echo "[bringup] Nav2 not active yet — retry STARTUP via lifecycle manager"
    timeout 15 ros2 service call /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' >/dev/null 2>&1 || true
    sleep 3
    elapsed=$((elapsed + 3))
done

wait "$LAUNCH_PID"
