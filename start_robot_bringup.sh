#!/bin/bash
# Headless ROS 2 bringup for plug-and-play boot.
# Starts Nav2, SLAM, LiDAR, ESP32 (micro-ROS), USB cam, YOLO pose.
# Does not start RViz — visualize from a remote PC.
# Do not use `set -u`: /opt/ros/jazzy/setup.bash references unset AMENT_* vars.

export HOME=/home/rctrobot
export USER=rctrobot
cd "$HOME/radxa_amr_v1"

# Match interactive shell ROS networking (Single-board local loopback + SHM)
source /opt/ros/jazzy/setup.bash
source "$HOME/radxa_amr_v1/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset ROS_STATIC_PEERS

# Device path templates (prefer persistent by-id, fallback dynamically)
ESP32_DEV="${ESP32_DEV:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C36140582-if00}"
LIDAR_DEV="${LIDAR_DEV:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_fcefc1eb0664ef118210e1a9c169b110-if00-port0}"
CAM_DEV="${CAM_DEV:-/dev/my_camera}"

# ------------------------------------------------------------------------------
# 1. Wait for NTP Time Synchronization (Short timeout for offline kiosk resilience)
# ------------------------------------------------------------------------------
wait_for_time_sync() {
    local timeout_s="${1:-5}"
    local elapsed=0
    echo "[bringup] Checking system clock synchronization via NTP (timeout: ${timeout_s}s)..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
            echo "[bringup] System clock synchronized via NTP (current time: $(date))"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[bringup] NTP sync skipped or offline — proceeding with current clock: $(date)"
    return 0
}

# ------------------------------------------------------------------------------
# 2. Wait for Hardware Serial & Video Devices (Dynamic fallback resolution)
# ------------------------------------------------------------------------------
wait_for_dev() {
    local primary="$1"
    local fallback_glob="$2"
    local label="$3"
    local timeout_s="${4:-60}"
    local elapsed=0
    echo "[bringup] Waiting for ${label} (target: ${primary}, fallback: ${fallback_glob})..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if [ -e "$primary" ]; then
            RESOLVED_DEV="$primary"
            echo "[bringup] ${label} found: ${RESOLVED_DEV}"
            return 0
        fi
        if [ -n "$fallback_glob" ]; then
            for f in $fallback_glob; do
                if [ -e "$f" ]; then
                    RESOLVED_DEV="$f"
                    echo "[bringup] ${label} resolved via fallback: ${RESOLVED_DEV}"
                    return 0
                fi
            done
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    RESOLVED_DEV="$primary"
    echo "[bringup] WARNING: ${label} not found after ${timeout_s}s — launching with: ${RESOLVED_DEV}"
    return 1
}

# Run synchronization and device discovery
wait_for_time_sync 5

wait_for_dev "$ESP32_DEV" "/dev/ttyACM0 /dev/ttyACM1" "ESP32 (micro-ROS)" 30
ESP32_DEV="$RESOLVED_DEV"

wait_for_dev "$LIDAR_DEV" "/dev/ttyUSB0 /dev/ttyUSB1" "LiDAR" 30
LIDAR_DEV="$RESOLVED_DEV"

wait_for_dev "$CAM_DEV" "/dev/video0 /dev/video1" "USB camera" 10
CAM_DEV="$RESOLVED_DEV"

# Reset ESP32 via DTR/RTS toggle to guarantee clean micro-ROS handshake with accurate host time
if [ -e "$ESP32_DEV" ]; then
    echo "[bringup] Pulsing DTR/RTS to restart ESP32 for fresh time sync..."
    python3 -c "import serial, time; s=serial.Serial('$ESP32_DEV', 921600); s.dtr=False; s.rts=True; time.sleep(0.1); s.dtr=True; s.rts=False; time.sleep(0.3); s.close()" 2>/dev/null || true
fi

# ------------------------------------------------------------------------------
# 3. Launch Main Robot Bringup Stack
# ------------------------------------------------------------------------------
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
    if timeout 8 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q 'success=True'; then
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
