#!/bin/bash
# Receptionist kiosk X session.
# GDM auto-login starts this instead of GNOME so the video is fullscreen
# as soon as the board boots — no extra command needed.

export DISPLAY="${DISPLAY:-:0}"
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=ReceptionistKiosk

# Black background, no screensaver / screen blanking
xsetroot -solid black 2>/dev/null || true
xset s off 2>/dev/null || true
xset s noblank 2>/dev/null || true
xset -dpms 2>/dev/null || true

# Clean exit handler
cleanup() {
    pkill -P $$ 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

# Keep X11 session alive indefinitely so play_video.service can render on :0
exec sleep infinity


