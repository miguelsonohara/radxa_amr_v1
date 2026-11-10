#!/usr/bin/env bash
# Install & Enable systemd services for Single-Board BPI-CM4-IO AMR:
# 1. robot_agent.service   (Port 5001 REST Daemon)
# 2. robot_bringup.service (Core ROS 2 Navigation/LiDAR/Firmware Bringup)

set -euo pipefail

WS_DIR="/home/rctrobot/radxa_amr_v1"

echo "=== Installing Robot Systemd Services for BPI-CM4-IO ==="

# 1. Copy service units to /etc/systemd/system/
sudo cp "${WS_DIR}/robot_agent.service" /etc/systemd/system/robot_agent.service
sudo cp "${WS_DIR}/robot_bringup.service" /etc/systemd/system/robot_bringup.service

# 2. Reload systemd daemon
sudo systemctl daemon-reload

# 3. Enable and start robot_agent.service
sudo systemctl enable --now robot_agent.service
echo "[OK] robot_agent.service enabled and started on http://127.0.0.1:5001"

# 4. Check status
sudo systemctl status robot_agent.service --no-pager || true

echo "=========================================================="
echo "To control robot_bringup.service:"
echo "  sudo systemctl start robot_bringup.service"
echo "  sudo systemctl stop robot_bringup.service"
echo "  sudo systemctl restart robot_bringup.service"
echo "  journalctl -u robot_bringup.service -f"
echo "=========================================================="
