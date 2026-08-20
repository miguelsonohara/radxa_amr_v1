#!/usr/bin/env python3
"""
Wi-Fi Receiver Daemon for Radxa 2
Listens on port 5001 over Ethernet (10.254.254.2) for Wi-Fi credentials sent from Radxa 1.
Connects Radxa 2 to the requested Wi-Fi using nmcli.
"""

import http.server
import json
import socketserver
import subprocess
from pathlib import Path

PORT = 5001


def run_cmd(cmd, timeout=30):
    try:
        res = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return res.returncode == 0, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def get_current_wifi():
    ok, out, _ = run_cmd("nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev")
    if ok and out:
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 4 and parts[1] == "wifi" and parts[2] == "connected":
                return parts[3]
    return ""


class ReceiverHandler(http.server.BaseHTTPRequestHandler):
    def _send_json(self, data, code=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/api/connect-wifi":
            content_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_len) if content_len > 0 else b"{}"

            try:
                data = json.loads(body.decode("utf-8"))
            except Exception:
                data = {}

            ssid = data.get("ssid", "").strip()
            password = data.get("password", "").strip()

            if not ssid:
                self._send_json({"success": False, "message": "SSID is required"}, 400)
                return

            current_ssid = get_current_wifi()
            if current_ssid == ssid:
                print(f"[Receiver] Radxa 2 is already connected to Wi-Fi: '{ssid}'")
                self._send_json({
                    "success": True,
                    "message": f"Radxa 2 is already connected to '{ssid}'",
                    "already_connected": True,
                })
                return

            print(f"[Receiver] Received request to connect to Wi-Fi: {ssid}")

            if password:
                cmd = f"nmcli dev wifi connect '{ssid}' password '{password}'"
            else:
                cmd = f"nmcli dev wifi connect '{ssid}'"

            ok, out, err = run_cmd(cmd, timeout=30)
            if not ok and ("privilege" in err.lower() or "privilege" in out.lower() or "permission" in err.lower()):
                print(f"[Receiver] Retrying connection with sudo nmcli...")
                if password:
                    cmd_sudo = f"sudo nmcli dev wifi connect '{ssid}' password '{password}'"
                else:
                    cmd_sudo = f"sudo nmcli dev wifi connect '{ssid}'"
                ok, out, err = run_cmd(cmd_sudo, timeout=30)

            if ok:
                print(f"[Receiver] Successfully connected Radxa 2 to {ssid}")
                self._send_json({"success": True, "message": f"Radxa 2 connected to {ssid}"})
            else:
                print(f"[Receiver] Connection failed: {err or out}")
                self._send_json({"success": False, "message": err or out or "Failed to connect"}, 500)
        else:
            self._send_json({"success": False, "message": "Not found"}, 404)


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def main():
    with ThreadedTCPServer(("0.0.0.0", PORT), ReceiverHandler) as httpd:
        print(f"Radxa 2 Wi-Fi Receiver listening on http://0.0.0.0:{PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down Wi-Fi Receiver.")


if __name__ == "__main__":
    main()
