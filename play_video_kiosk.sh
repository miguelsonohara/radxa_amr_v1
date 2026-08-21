#!/bin/bash
export DISPLAY="${DISPLAY:-:0}"

VIDEO="${1:-/home/radxa/receptionist_robot_ws/xoaylai180.mp4}"
IPC_SOCKET="/tmp/mpvsocket"

# Wait for X display to be ready
until xset q >/dev/null 2>&1; do
    sleep 0.5
done

# If GNOME is present, close Overview
if pgrep -x gnome-shell >/dev/null 2>&1; then
    sleep 1
    xdotool key Escape 2>/dev/null || true
fi

# Disable blanking / DPMS while the player is running
xset s off 2>/dev/null || true
xset s noblank 2>/dev/null || true
xset -dpms 2>/dev/null || true

# Remove stale IPC socket if exists
rm -f "$IPC_SOCKET"

# Run MPV with rock-solid CPU decoding and minimal GPU shader overhead
exec /usr/bin/mpv \
    --hwdec=no \
    --vd-lavc-threads=4 \
    --vo=gpu \
    --gpu-context=x11egl \
    --profile=fast \
    --scale=bilinear \
    --cscale=bilinear \
    --dscale=bilinear \
    --dither-depth=no \
    --correct-downscaling=no \
    --sigmoid-upscaling=no \
    --x11-bypass-compositor=yes \
    --force-window=immediate \
    --fullscreen \
    --fs-screen=0 \
    --geometry=100%x100%+0+0 \
    --no-border \
    --ontop \
    --ontop-level=system \
    --no-osd-bar \
    --cursor-autohide=always \
    --stop-screensaver=yes \
    --input-default-bindings=no \
    --input-vo-keyboard=no \
    --keep-open=always \
    --loop-file=inf \
    --panscan=1.0 \
    --input-ipc-server="$IPC_SOCKET" \
    "$VIDEO"

