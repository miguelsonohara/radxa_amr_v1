#!/bin/bash
export DISPLAY="${DISPLAY:-:0}"

VIDEO="${1:-/home/radxa/receptionist_robot_ws/xoaylai180.mp4}"
IPC_SOCKET="/tmp/mpvsocket"
VENUS_DEC="/dev/video3"

# Safety net only: --loop-playlist avoids the seek that wedges Venus, but if the
# decoder ever stalls anyway, exiting non-zero lets systemd rebuild it.
STALL_POLL_S=10
STALL_LIMIT=3

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

# Offload H.264 to the Venus block. Software decode of this 1080p/15Mbps clip
# costs ~0.75 of a core and starves the micro-ROS serial reader and the Nav2
# controller loop, which shows up as jerky motion. Looping via the playlist
# rather than --loop-file is required: an in-place loop seek deadlocks the
# v4l2m2m queues on the first wrap.
if [ -c "$VENUS_DEC" ]; then
    DECODE_ARGS=(--hwdec=v4l2m2m-copy)
else
    DECODE_ARGS=(--hwdec=no --vd-lavc-threads=2)
fi

/usr/bin/mpv \
    "${DECODE_ARGS[@]}" \
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
    --loop-playlist=inf \
    --panscan=1.0 \
    --msg-level=all=warn \
    --term-status-msg= \
    --input-ipc-server="$IPC_SOCKET" \
    "$VIDEO" &
MPV_PID=$!

cleanup() {
    kill "$MPV_PID" 2>/dev/null || true
    wait "$MPV_PID" 2>/dev/null || true
    rm -f "$IPC_SOCKET"
}
trap cleanup EXIT INT TERM

query_playback_time() {
    python3 - "$IPC_SOCKET" <<'PY' 2>/dev/null
import json, socket, sys

REQ_ID = 7391

try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect(sys.argv[1])
    s.sendall(json.dumps(
        {"command": ["get_property", "playback-time"], "request_id": REQ_ID}
    ).encode() + b"\n")

    # mpv interleaves unsolicited event messages with command replies, so match
    # on request_id rather than assuming the first line is our answer.
    buf = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            sys.exit(1)
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("request_id") != REQ_ID:
                continue
            if msg.get("error") != "success":
                sys.exit(1)
            print(msg["data"])
            sys.exit(0)
except SystemExit:
    raise
except Exception:
    sys.exit(1)
PY
}

last_pos=""
stalled=0
unreachable=0
while kill -0 "$MPV_PID" 2>/dev/null; do
    sleep "$STALL_POLL_S"

    if ! pos="$(query_playback_time)" || [ -z "$pos" ]; then
        # A wedged decoder also stops servicing the IPC socket, so treat a
        # persistently silent player as a hang rather than ignoring it.
        unreachable=$((unreachable + 1))
        if [ "$unreachable" -ge "$STALL_LIMIT" ]; then
            echo "[kiosk] player unresponsive on IPC — restarting" >&2
            exit 1
        fi
        continue
    fi
    unreachable=0

    if [ "$pos" = "$last_pos" ]; then
        stalled=$((stalled + 1))
        echo "[kiosk] playback stalled at ${pos}s (${stalled}/${STALL_LIMIT})" >&2
        if [ "$stalled" -ge "$STALL_LIMIT" ]; then
            echo "[kiosk] decoder deadlock detected — restarting player" >&2
            exit 1
        fi
    else
        stalled=0
    fi
    last_pos="$pos"
done

wait "$MPV_PID"
