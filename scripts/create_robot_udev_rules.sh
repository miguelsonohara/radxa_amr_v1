#!/usr/bin/env bash
# ==============================================================================
# Setup Udev Rules for Receptionist Robot (ESP32, LiDAR, Camera)
# Creates persistent symlinks:
#   /dev/robot_esp32  -> Any CH340/CH341/CH343 (1a86) or Espressif (303a)
#   /dev/robot_lidar  -> Any CP2102/CP2102N (10c4:ea60) or RPLidar
#   /dev/rplidar      -> Alias for LiDAR
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_FILE="/etc/udev/rules.d/99-receptionist-robot.rules"

echo "=== Installing Receptionist Robot Udev Rules ==="

sudo bash -c "cat << 'EOF' > '$UDEV_FILE'
# 1. ESP32-S3 Motor Controller (Any CH340/CH341/CH343/WCH or Espressif USB CDC)
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", MODE:=\"0777\", SYMLINK+=\"robot_esp32\"
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"303a\", MODE:=\"0777\", SYMLINK+=\"robot_esp32\"

# 2. LiDAR Sensor (Silicon Labs CP2102/CP2102N used by RPLidar and HCLiDAR)
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0777\", SYMLINK+=\"robot_lidar rplidar\"
EOF"

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Adding user '$USER' to dialout and video groups..."
sudo usermod -aG dialout,video "$USER" || true

echo "=== Udev rules successfully installed ==="
echo "Persistent links created:"
echo "  - ESP32: /dev/robot_esp32"
echo "  - LiDAR: /dev/robot_lidar (/dev/rplidar)"
