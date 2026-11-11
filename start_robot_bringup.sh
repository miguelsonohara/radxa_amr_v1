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

# ------------------------------------------------------------------------------
# 1. Dynamic Device Path Discovery
# Auto-detects ESP32, LiDAR, and Camera even when swapping hardware (different serial IDs)
# ------------------------------------------------------------------------------
get_udev_vid() {
    local dev="$1"
    udevadm info -q property -n "$dev" 2>/dev/null | grep -E '^ID_VENDOR_ID=' | cut -d'=' -f2 | tr '[:upper:]' '[:lower:]'
}

# True if candidate exists and is NOT the same physical node as $exclude
# (compares resolved paths, so /dev/serial/by-id/... and /dev/ttyUSB0 are treated as identical)
dev_usable() {
    local cand="$1"
    local exclude="${2:-}"
    [ -e "$cand" ] || return 1
    [ -z "$exclude" ] && return 0
    [ "$(readlink -f "$cand")" != "$(readlink -f "$exclude")" ]
}

# Vendor IDs: ESP32 bridges = 1a86 (QinHeng CH34x) / 303a (Espressif native USB); LiDAR = 10c4 (Silicon Labs CP210x)
ESP32_VIDS="1a86 303a"
LIDAR_VIDS="10c4"

vid_in() {
    local vid="$1"; shift
    for v in "$@"; do [ "$vid" = "$v" ] && return 0; done
    return 1
}

find_esp32_dev() {
    local exclude="${1:-}"
    # 1. Persistent udev symlink
    if dev_usable "/dev/robot_esp32" "$exclude"; then echo "/dev/robot_esp32"; return 0; fi
    # 2. Match by-id wildcards (QinHeng CH340/CH341/CH343/WCH 1a86, Espressif 303a)
    for dev in /dev/serial/by-id/*1a86* /dev/serial/by-id/*wch* /dev/serial/by-id/*Espressif* /dev/serial/by-id/*Single_Serial*; do
        if dev_usable "$dev" "$exclude"; then echo "$dev"; return 0; fi
    done
    # 3. Any ttyACM*/ttyUSB* whose vendor ID is a known ESP32 bridge
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        if dev_usable "$dev" "$exclude" && vid_in "$(get_udev_vid "$dev")" $ESP32_VIDS; then
            echo "$dev"; return 0
        fi
    done
    # 4. Fallback: any ttyACM*/ttyUSB* that is NOT a LiDAR bridge (never steal the LiDAR port)
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        if dev_usable "$dev" "$exclude" && ! vid_in "$(get_udev_vid "$dev")" $LIDAR_VIDS; then
            echo "$dev"; return 0
        fi
    done
    return 1
}

find_lidar_dev() {
    local exclude="${1:-}"
    # 1. Persistent udev symlinks
    for symlink in "/dev/robot_lidar" "/dev/rplidar"; do
        if dev_usable "$symlink" "$exclude"; then echo "$symlink"; return 0; fi
    done
    # 2. Match by-id wildcards for Silicon Labs CP210x (used by RPLidar & HCLiDAR)
    for dev in /dev/serial/by-id/*CP210* /dev/serial/by-id/*Silicon_Labs* /dev/serial/by-id/*RPLIDAR* /dev/serial/by-id/*rplidar*; do
        if dev_usable "$dev" "$exclude"; then echo "$dev"; return 0; fi
    done
    # 3. Any ttyUSB* whose vendor ID is Silicon Labs
    for dev in /dev/ttyUSB*; do
        if dev_usable "$dev" "$exclude" && vid_in "$(get_udev_vid "$dev")" $LIDAR_VIDS; then
            echo "$dev"; return 0
        fi
    done
    # 4. Fallback: any ttyUSB* that is NOT an ESP32 bridge (never steal the ESP32 port)
    for dev in /dev/ttyUSB*; do
        if dev_usable "$dev" "$exclude" && ! vid_in "$(get_udev_vid "$dev")" $ESP32_VIDS; then
            echo "$dev"; return 0
        fi
    done
    return 1
}

wait_for_esp32() {
    local timeout_s="${1:-30}"
    local elapsed=0
    echo "[bringup] Searching dynamically for ESP32 (CH340/1a86, Espressif, ttyACM/ttyUSB)..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        local dev
        dev=$(find_esp32_dev)
        if [ -n "$dev" ] && [ -e "$dev" ]; then
            echo "[bringup] ESP32 detected: $dev"
            ESP32_DEV="$dev"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    ESP32_DEV="${ESP32_DEV:-/dev/ttyACM0}"
    echo "[bringup] WARNING: ESP32 not detected after ${timeout_s}s — defaulting to: $ESP32_DEV"
    return 1
}

wait_for_lidar() {
    local timeout_s="${1:-30}"
    local elapsed=0
    echo "[bringup] Searching dynamically for LiDAR (CP210x/10c4, RPLidar, Silicon Labs)..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        local dev
        dev=$(find_lidar_dev "$ESP32_DEV")
        if [ -n "$dev" ] && [ -e "$dev" ]; then
            echo "[bringup] LiDAR detected: $dev"
            LIDAR_DEV="$dev"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    LIDAR_DEV="${LIDAR_DEV:-/dev/ttyUSB0}"
    echo "[bringup] WARNING: LiDAR not detected after ${timeout_s}s — defaulting to: $LIDAR_DEV"
    return 1
}

wait_for_camera() {
    local timeout_s="${1:-10}"
    local elapsed=0
    echo "[bringup] Searching dynamically for Camera (/dev/my_camera, /dev/video*)..."
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if [ -e "/dev/my_camera" ]; then
            CAM_DEV="/dev/my_camera"
            echo "[bringup] Camera detected: $CAM_DEV"
            return 0
        fi
        for v in /dev/video0 /dev/video1; do
            if [ -e "$v" ]; then
                CAM_DEV="$v"
                echo "[bringup] Camera detected: $CAM_DEV"
                return 0
            fi
        done
        sleep 1
        elapsed=$((elapsed + 1))
    done
    CAM_DEV="${CAM_DEV:-/dev/video0}"
    echo "[bringup] WARNING: Camera not detected after ${timeout_s}s — defaulting to: $CAM_DEV"
    return 1
}

# ------------------------------------------------------------------------------
# 2. Wait for NTP Time Synchronization (Short timeout for offline kiosk resilience)
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

# Run synchronization and device discovery
wait_for_time_sync 5
wait_for_esp32 30
wait_for_lidar 30
wait_for_camera 10

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
