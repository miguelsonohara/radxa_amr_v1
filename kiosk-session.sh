#!/bin/bash
# Receptionist kiosk X session.
# GDM auto-login starts this instead of GNOME, so the board comes up with a bare
# X display and none of the GNOME shell overhead competing with the robot stack.
# The video itself is owned by play_video.service (system unit), which is CPU
# isolated to the little cores; this script only has to keep :0 alive.

export DISPLAY="${DISPLAY:-:0}"
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=ReceptionistKiosk

# Black background, no screensaver / screen blanking
xsetroot -solid black 2>/dev/null || true
xset s off 2>/dev/null || true
xset s noblank 2>/dev/null || true
xset -dpms 2>/dev/null || true

cleanup() {
    pkill -P $$ 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

# Hold the session open. Backgrounding `sleep` instead of exec'ing it keeps this
# shell alive as PID of the session so the trap above can still run on logout.
sleep infinity &
wait $!
