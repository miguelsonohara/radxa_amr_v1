#!/bin/bash
# ==============================================================================
# Script: setup_ethernet_static.sh
# Purpose: Auto-configure static IP 10.254.254.2/24 on ANY physical Ethernet port
#          for Radxa Dragon Q6A (even after cloning SD card across boards with different MACs).
# ==============================================================================

set -e

TARGET_IP="10.254.254.2/24"
CONN_NAME="Wired connection 1"

echo "[eth-setup] Detecting physical Ethernet interface..."
ETH_DEV=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^(en|eth|end)' | head -n 1)

if [ -z "$ETH_DEV" ]; then
    echo "[eth-setup] ERROR: No physical Ethernet interface found on this board!"
    exit 1
fi
echo "[eth-setup] Found Ethernet device: ${ETH_DEV}"

# Ensure connection profile exists in NetworkManager
if ! nmcli connection show "${CONN_NAME}" >/dev/null 2>&1; then
    echo "[eth-setup] Creating new connection profile: '${CONN_NAME}'..."
    nmcli connection add type ethernet con-name "${CONN_NAME}" ifname "${ETH_DEV}"
fi

echo "[eth-setup] Configuring universal match and static IP ${TARGET_IP}..."
nmcli connection modify "${CONN_NAME}" \
    connection.interface-name "" \
    802-3-ethernet.mac-address "" \
    802-3-ethernet.cloned-mac-address "" \
    802-3-ethernet.auto-negotiate yes \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    ipv4.method manual \
    ipv4.addresses "${TARGET_IP}"

echo "[eth-setup] Activating connection on ${ETH_DEV}..."
nmcli device connect "${ETH_DEV}" 2>/dev/null || nmcli connection up "${CONN_NAME}" 2>/dev/null || true

echo "[eth-setup] Current IP status:"
hostname -I
echo "[eth-setup] Ethernet static IP setup complete!"
