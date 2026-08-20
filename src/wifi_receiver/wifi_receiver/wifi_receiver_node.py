#!/usr/bin/env python3
"""
Wi-Fi Receiver ROS 2 Node for Radxa 1 (Head Host Board)
Listens on HTTP port 5001 (configurable via parameter) over Ethernet for Wi-Fi credentials sent from Radxa 2.
Connects Radxa 1 to the requested Wi-Fi using nmcli and publishes status to ROS 2 topic.
"""

import http.server
import json
import socketserver
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def run_cmd(cmd, timeout=30):
    try:
        res = subprocess.run(
            cmd,
            shell=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return res.returncode == 0, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def make_receiver_handler(ros_node):
    class ReceiverHandler(http.server.BaseHTTPRequestHandler):
        def _send_json(self, data, code=200):
            body = json.dumps(data).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

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

                ros_node.get_logger().info(f"Received request to connect to Wi-Fi: '{ssid}'")

                if password:
                    cmd = ["nmcli", "dev", "wifi", "connect", ssid, "password", password]
                else:
                    cmd = ["nmcli", "dev", "wifi", "connect", ssid]

                ok, out, err = run_cmd(cmd, timeout=30)
                if ok:
                    msg_text = f"Successfully connected Radxa 1 to '{ssid}'"
                    ros_node.get_logger().info(msg_text)
                    ros_node.publish_wifi_status(True, ssid, msg_text)
                    self._send_json({"success": True, "message": msg_text})
                else:
                    err_msg = err or out or "Failed to connect"
                    ros_node.get_logger().error(f"Connection to '{ssid}' failed: {err_msg}")
                    ros_node.publish_wifi_status(False, ssid, err_msg)
                    self._send_json({"success": False, "message": err_msg}, 500)
            else:
                self._send_json({"success": False, "message": "Not found"}, 404)

        def log_message(self, format, *args):
            # Suppress default HTTP request logging to keep ROS 2 logs clean
            pass

    return ReceiverHandler


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


class WifiReceiverNode(Node):
    def __init__(self):
        super().__init__('wifi_receiver_node')

        self.declare_parameter('port', 5001)
        self.declare_parameter('wifi_status_topic', '/wifi_status')

        self.port = self.get_parameter('port').value
        status_topic = self.get_parameter('wifi_status_topic').value

        self.status_pub = self.create_publisher(String, status_topic, 10)

        handler_class = make_receiver_handler(self)
        self.httpd = ThreadedTCPServer(("0.0.0.0", self.port), handler_class)

        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"Wi-Fi Receiver Node initialized. HTTP server listening on 0.0.0.0:{self.port}, "
            f"publishing status to topic '{status_topic}'."
        )

    def publish_wifi_status(self, success, ssid, message):
        status_msg = String()
        status_msg.data = json.dumps({
            "success": success,
            "ssid": ssid,
            "message": message
        })
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down Wi-Fi Receiver HTTP server...")
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception as e:
            self.get_logger().warn(f"Error during HTTP server shutdown: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WifiReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Wi-Fi Receiver Node shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
