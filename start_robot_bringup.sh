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
export ROS_STATIC_PEERS="10.254.254.1"

ESP32_DEV="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C36140582-if00"
LIDAR_DEV="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_fcefc1eb0664ef118210e1a9c169b110-if00-port0"
CAM_DEV="/dev/my_camera"

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
    s = serial.Serial()
    s.dtr = False
    s.rts = True
    time.sleep(0.15)
    s.rts = False
    s.dtr = False
    s.close()
    print([bringup] ESP32 normal run-mode reset complete.)
except Exception as e:
    print(f[bringup] WARNING: ESP32 reset failed: {e})
" 2>/dev/null || true
        sleep 0.3
    fi
}

wait_for_dev "$ESP32_DEV" "ESP32 (micro-ROS)" 90
wait_for_dev "$LIDAR_DEV" "LiDAR" 90
wait_for_dev "$CAM_DEV" "USB camera" 30

# Reset ESP32 hardware cleanly so it synchronizes clock with Linux host immediately
reset_esp32_hardware "$ESP32_DEV"

ros2 launch receptionist_robot_bringup bringup.launch.py \
    serial_port:="$ESP32_DEV" \
    lidar_port:="$LIDAR_DEV" \
    camera_device:="$CAM_DEV" \
    use_sim_time:=false &
LAUNCH_PID=$!

echo "[bringup] waiting for ESP32 /odom before confirming Nav2..."
elapsed=0
while [ "$elapsed" -lt 120 ]; do
    if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1; then
        echo "[bringup] /odom is publishing"
        break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done

echo "[bringup] ensuring Nav2 lifecycle is active"
elapsed=0
while [ "$elapsed" -lt 90 ]; do
    if timeout 8 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q success=True; then
        echo "[bringup] Nav2 is active"
        break
    fi
    echo "[bringup] Nav2 not active yet — retry STARTUP"
    timeout 25 ros2 service call /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' >/dev/null 2>&1 || true
    sleep 3
    elapsed=$((elapsed + 3))
done

wait "$LAUNCH_PID"
