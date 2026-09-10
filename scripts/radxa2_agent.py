#!/usr/bin/env python3
"""
radxa2_agent.py - Robot Navigation & Motion Controller Agent Daemon
Runs on Radxa Tầng Dưới (10.254.254.2), listening on port 5001.
Handles HTTP REST API requests from Radxa Tầng Trên (10.254.254.1):
  - POST /api/robot/mode      : Switch between mapping and localization modes
  - POST /api/robot/save-map  : Serialize posegraph & save 2D occupancy grid
  - POST /api/robot/home      : Record current TF pose or command robot to return Home
  - GET  /api/robot/status    : Retrieve current operational status, mode, and map info
"""

import http.server
import json
import math
import os
import re
import socketserver
import subprocess
import threading
import time

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from slam_toolbox.srv import SerializePoseGraph, SaveMap
from std_msgs.msg import String
import tf2_ros

# Central paths
WS_DIR = "/home/radxa/receptionist_robot_ws"
CONFIG_FILE = os.path.join(WS_DIR, "src/receptionist_robot_bringup/config/slam_toolbox_params.yaml")
SYMLINK_CONFIG = os.path.join(WS_DIR, "slam_toolbox_params.yaml")
MAPS_DIR = os.path.join(WS_DIR, "map")
HOME_POSE_FILE = "/home/radxa/home_pose.json"
SERVICE_NAME = "robot_bringup.service"
PORT = 5001


def get_stored_home_pose():
    """Reads home_pose from ~/home_pose.json or defaults to [0.0, 0.0, 0.0]."""
    if os.path.exists(HOME_POSE_FILE):
        try:
            with open(HOME_POSE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                if "home_pose" in data and isinstance(data["home_pose"], list):
                    return [float(x) for x in data["home_pose"]]
                if "x" in data and "y" in data:
                    return [float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("yaw", 0.0))]
        except Exception as e:
            print(f"[radxa2_agent] Error reading {HOME_POSE_FILE}: {e}")
    return [0.0, 0.0, 0.0]


def save_stored_home_pose(pose_list):
    """Saves [x, y, yaw] to ~/home_pose.json."""
    data = {
        "home_pose": [float(pose_list[0]), float(pose_list[1]), float(pose_list[2])],
        "x": float(pose_list[0]),
        "y": float(pose_list[1]),
        "yaw": float(pose_list[2]),
        "timestamp": time.time()
    }
    try:
        with open(HOME_POSE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return True
    except Exception as e:
        print(f"[radxa2_agent] Error writing {HOME_POSE_FILE}: {e}")
        return False


def update_slam_params(mode, map_file_name=None):
    """
    Updates mode and map_file_name in slam_toolbox_params.yaml preserving comments.
    """
    target_file = CONFIG_FILE
    if not os.path.exists(target_file) and os.path.exists(SYMLINK_CONFIG):
        target_file = SYMLINK_CONFIG

    if not os.path.exists(target_file):
        raise FileNotFoundError(f"Cannot find SLAM config file at {target_file}")

    with open(target_file, "r", encoding="utf-8") as f:
        content = f.read()

    # 1. Update mode (match active 'mode: <val>' line, not comments like '# mode: mapping')
    pattern_mode = r'(^[ \t]*mode:[ \t]*)[^\r\n#]+'
    if re.search(pattern_mode, content, flags=re.MULTILINE):
        content = re.sub(pattern_mode, r'\g<1>' + mode, content, flags=re.MULTILINE)
    else:
        # If not present, add under ros__parameters
        content = re.sub(r'(ros__parameters:\s*\n)', r'\1    mode: ' + mode + '\n', content)

    # 2. Handle map_file_name based on mode
    if mode == "mapping":
        # In fresh mapping mode, comment out any active map_file_name so SLAM starts an empty map!
        pattern_map = r'(^[ \t]*)(map_file_name:[ \t]*[^\r\n#]+)'
        content = re.sub(pattern_map, r'\g<1># \g<2>', content, flags=re.MULTILINE)
    elif mode == "localization" and map_file_name:
        # In localization mode, ensure map_file_name is active and points to the map
        pattern_map_active = r'(^[ \t]*)(map_file_name:[ \t]*)[^\r\n#]+'
        pattern_map_commented = r'(^[ \t]*)#[ \t]*(map_file_name:[ \t]*)[^\r\n#]+'
        if re.search(pattern_map_active, content, flags=re.MULTILINE):
            content = re.sub(pattern_map_active, r'\g<1>\g<2>' + map_file_name, content, flags=re.MULTILINE)
        elif re.search(pattern_map_commented, content, flags=re.MULTILINE):
            content = re.sub(pattern_map_commented, r'\g<1>\g<2>' + map_file_name, content, flags=re.MULTILINE)
        else:
            content = re.sub(r'(^[ \t]*mode:[ \t]*[^\r\n]+)', r'\1\n    map_file_name: ' + map_file_name, content, flags=re.MULTILINE)

    with open(target_file, "w", encoding="utf-8") as f:
        f.write(content)


def read_slam_params():
    """Reads current mode and map_file_name from yaml."""
    target_file = CONFIG_FILE
    if not os.path.exists(target_file) and os.path.exists(SYMLINK_CONFIG):
        target_file = SYMLINK_CONFIG

    mode = "unknown"
    map_file_name = ""
    if os.path.exists(target_file):
        with open(target_file, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                if stripped.startswith("mode:"):
                    mode = stripped.split(":", 1)[1].strip()
                elif stripped.startswith("map_file_name:"):
                    map_file_name = stripped.split(":", 1)[1].strip()
    return mode, map_file_name


LAST_SAVED_MAP_FILE = os.path.join(WS_DIR, "last_saved_map.txt")

def set_last_saved_map(name: str):
    try:
        clean_name = os.path.basename(name).strip()
        for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
            if clean_name.endswith(ext):
                clean_name = clean_name[:-len(ext)]
        with open(LAST_SAVED_MAP_FILE, "w", encoding="utf-8") as f:
            f.write(clean_name)
    except Exception as e:
        print(f"[radxa2_agent] Error saving last_saved_map: {e}")

def get_last_saved_map() -> str:
    if os.path.exists(LAST_SAVED_MAP_FILE):
        try:
            with open(LAST_SAVED_MAP_FILE, "r", encoding="utf-8") as f:
                return f.read().strip()
        except Exception:
            pass
    return ""

def scan_available_maps():
    """Scans MAPS_DIR and returns sorted list of valid map metadata (newest first)."""
    os.makedirs(MAPS_DIR, exist_ok=True)
    maps_dict = {}

    for fname in os.listdir(MAPS_DIR):
        fpath = os.path.join(MAPS_DIR, fname)
        if not os.path.isfile(fpath):
            continue
        base, ext = os.path.splitext(fname)
        ext = ext.lower()
        if ext not in [".posegraph", ".data", ".yaml", ".pgm"]:
            continue
        if base in ["keepout_mask"]:
            continue

        if base not in maps_dict:
            mtime = os.path.getmtime(fpath)
            maps_dict[base] = {
                "name": base,
                "full_path": os.path.join(MAPS_DIR, base),
                "has_posegraph": False,
                "has_yaml": False,
                "modified": mtime,
                "modified_str": time.strftime("%d/%m/%Y %H:%M", time.localtime(mtime)),
                "total_bytes": 0
            }

        m = maps_dict[base]
        if ext == ".posegraph":
            m["has_posegraph"] = True
        elif ext == ".yaml":
            m["has_yaml"] = True
        m["total_bytes"] += os.path.getsize(fpath)
        f_mtime = os.path.getmtime(fpath)
        if f_mtime > m["modified"]:
            m["modified"] = f_mtime
            m["modified_str"] = time.strftime("%d/%m/%Y %H:%M", time.localtime(f_mtime))

    map_list = list(maps_dict.values())
    for m in map_list:
        m["size_mb"] = round(m["total_bytes"] / (1024 * 1024), 2)

    map_list.sort(key=lambda x: x["modified"], reverse=True)
    return map_list


def control_bringup_service(action: str):
    """Starts, stops, or restarts robot_bringup.service."""
    action = action.strip().lower()
    if action not in ["start", "stop", "restart", "status"]:
        return False, f"Hành động không hợp lệ: '{action}'. Chỉ hỗ trợ: 'start', 'stop', 'restart', 'status'"
    
    if action == "status":
        res = subprocess.run(["systemctl", "is-active", SERVICE_NAME], capture_output=True, text=True)
        is_active = (res.stdout.strip() == "active")
        return is_active, res.stdout.strip()

    if action == "restart":
        restart_bringup_service()
        return True, f"Đang khởi động lại dịch vụ {SERVICE_NAME}..."

    cmd = ["sudo", "systemctl", action, SERVICE_NAME]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0:
        return True, f"Đã thực hiện {action} dịch vụ {SERVICE_NAME}"
    else:
        return False, res.stderr.strip() or f"Lỗi {action} {SERVICE_NAME}"


def restart_bringup_service():
    """Restarts robot_bringup.service in a background thread."""
    def _restart():
        print(f"[radxa2_agent] Restarting {SERVICE_NAME} via systemctl in background...")
        subprocess.run(["sudo", "systemctl", "restart", SERVICE_NAME], check=False)
        print(f"[radxa2_agent] Background restart of {SERVICE_NAME} completed.")

    t = threading.Thread(target=_restart, daemon=True)
    t.start()


class Radxa2AgentNode(Node):
    def __init__(self):
        super().__init__('radxa2_agent_node')
        self.get_logger().info("Initializing Radxa 2 Agent ROS 2 Node...")

        # Publisher to /goal_pose for Nav2 navigation
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Publisher to /amr/command for Behavior Tree
        self.cmd_pub = self.create_publisher(String, '/amr/command', 10)

        # Live robot state from Behavior Tree via /general_status
        self.current_robot_state = "IDLE"
        self.create_subscription(String, '/general_status', self._on_general_status, 10)

        # Service clients for SLAM Toolbox serialize_map and save_map
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')

    def _on_general_status(self, msg: String):
        self.current_robot_state = msg.data.strip().upper()

    def send_amr_command(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Dispatched AMR command: '{cmd_str}'")

    def get_current_pose(self):
        """Returns stored home pose as fallback (TF listener removed to save CPU)."""
        p = get_stored_home_pose()
        return p[0], p[1], p[2]

    def publish_goal_pose(self, x, y, yaw):
        """Publishes a PoseStamped goal to /goal_pose."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(msg)
        self.get_logger().info(f"Published goal pose to /goal_pose: x={x}, y={y}, yaw={yaw}")

    def serialize_map_service(self, filepath_no_ext, timeout_sec=8.0):
        """Calls /slam_toolbox/serialize_map service."""
        if not self.serialize_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("Service /slam_toolbox/serialize_map not available")
            return False, "SLAM Toolbox serialize service not available"

        req = SerializePoseGraph.Request()
        req.filename = filepath_no_ext
        future = self.serialize_client.call_async(req)

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if future.done():
                try:
                    res = future.result()
                    if res.result == 0:  # RESULT_SUCCESS
                        return True, "Posegraph serialized successfully"
                    else:
                        return False, f"Serialize service returned failure code: {res.result}"
                except Exception as e:
                    return False, f"Exception during service call: {e}"
            time.sleep(0.1)

        return False, "Serialize service call timed out"

    def save_map_service(self, filepath_no_ext, timeout_sec=8.0):
        """Calls /slam_toolbox/save_map service to save .yaml and .pgm."""
        if not self.save_map_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("Service /slam_toolbox/save_map not available")
            return False, "SLAM Toolbox save_map service not available"

        req = SaveMap.Request()
        req.name = String()
        req.name.data = filepath_no_ext
        future = self.save_map_client.call_async(req)

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if future.done():
                try:
                    res = future.result()
                    if res.result == 0:  # RESULT_SUCCESS
                        return True, "Occupancy grid saved successfully"
                    else:
                        return False, f"SaveMap service returned failure code: {res.result}"
                except Exception as e:
                    return False, f"Exception during SaveMap call: {e}"
            time.sleep(0.1)

        return False, "SaveMap service call timed out"


def make_request_handler(ros_node: Radxa2AgentNode):
    class RequestHandler(http.server.BaseHTTPRequestHandler):
        def _send_json(self, data, code=200):
            body = json.dumps(data, indent=2).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            if self.path == "/api/robot/status":
                mode, current_map = read_slam_params()
                home_pose = get_stored_home_pose()

                # Check if robot_bringup.service is active
                res = subprocess.run(["systemctl", "is-active", SERVICE_NAME], capture_output=True, text=True)
                slam_active = (res.stdout.strip() == "active")

                # Optional: also check if SLAM node is discovered in ROS 2 graph
                node_names = ros_node.get_node_names()
                slam_node_alive = "slam_toolbox" in node_names or "async_slam_toolbox_node" in node_names
                bt_node_alive = "amr_bt_node" in node_names
                nav2_alive = "controller_server" in node_names and "bt_navigator" in node_names

                self._send_json({
                    "mode": mode,
                    "robot_state": getattr(ros_node, "current_robot_state", "IDLE"),
                    "home_pose": home_pose,
                    "current_map": current_map,
                    "current_map_name": os.path.basename(current_map) if current_map else "",
                    "last_saved_map": get_last_saved_map(),
                    "slam_active": slam_active or slam_node_alive,
                    "bt_active": bt_node_alive,
                    "nav2_active": nav2_alive,
                    "bringup_service_active": slam_active
                })
            elif self.path == "/api/robot/maps":
                _, current_map = read_slam_params()
                current_base = os.path.basename(current_map) if current_map else ""
                maps = scan_available_maps()
                self._send_json({
                    "success": True,
                    "current_map": current_base,
                    "current_map_path": current_map,
                    "last_saved_map": get_last_saved_map(),
                    "maps": maps
                })

            else:
                self._send_json({"success": False, "message": f"Endpoint '{self.path}' not found"}, 404)

        def do_POST(self):
            content_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_len) if content_len > 0 else b"{}"

            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception as e:
                self._send_json({"success": False, "message": f"Invalid JSON body: {e}"}, 400)
                return

            # ------------------------------------------------------------------
            # a) POST /api/robot/mode
            # ------------------------------------------------------------------
            if self.path == "/api/robot/mode":
                requested_mode = data.get("mode", "").strip().lower()
                if requested_mode not in ["mapping", "localization"]:
                    self._send_json({
                        "success": False,
                        "message": f"Invalid mode: '{requested_mode}'. Supported: 'mapping', 'localization'"
                    }, 400)
                    return

                if requested_mode == "mapping":
                    save_stored_home_pose([0.0, 0.0, 0.0])
                    update_slam_params("mapping")
                    restart_bringup_service()
                    self._send_json({
                        "success": True,
                        "mode": "mapping",
                        "home_pose": [0.0, 0.0, 0.0],
                        "message": "Switched to mapping mode. Restarting robot bringup service..."
                    })

                elif requested_mode == "localization":
                    map_file_name = (data.get("map_name") or data.get("map_file_name") or "").strip()
                    if not map_file_name:
                        # Check existing
                        _, existing_map = read_slam_params()
                        last_saved = get_last_saved_map()
                        map_file_name = last_saved or existing_map or os.path.join(MAPS_DIR, "demo_map1")

                    # Normalize map path (strip extension and ensure absolute path)
                    base_map = map_file_name
                    for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                        if base_map.endswith(ext):
                            base_map = base_map[:-len(ext)]
                    if not base_map.startswith("/"):
                        base_map = os.path.join(MAPS_DIR, base_map)

                    set_last_saved_map(os.path.basename(base_map))
                    update_slam_params("localization", base_map)
                    home_pose = get_stored_home_pose()
                    restart_bringup_service()
                    self._send_json({
                        "success": True,
                        "mode": "localization",
                        "home_pose": home_pose,
                        "map_file_name": base_map,
                        "message": f"Switched to localization mode using map '{base_map}'. Restarting robot bringup service..."
                    })

            # ------------------------------------------------------------------
            # a2) POST /api/robot/select-map
            # ------------------------------------------------------------------
            elif self.path == "/api/robot/select-map":
                map_name = (data.get("map_name") or data.get("map_file_name") or "").strip()
                should_restart = bool(data.get("restart", False))

                if not map_name:
                    self._send_json({"success": False, "message": "Missing map_name"}, 400)
                    return

                # Normalize map path
                base_map = map_name
                for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                    if base_map.endswith(ext):
                        base_map = base_map[:-len(ext)]
                if not base_map.startswith("/"):
                    base_map = os.path.join(MAPS_DIR, base_map)

                map_base_name = os.path.basename(base_map)
                set_last_saved_map(map_base_name)
                update_slam_params("localization", base_map)

                restarted = False
                if should_restart:
                    restart_bringup_service()
                    restarted = True

                self._send_json({
                    "success": True,
                    "current_map": map_base_name,
                    "map_file_name": base_map,
                    "restarted": restarted,
                    "message": f"Đã chọn bản đồ '{map_base_name}'" + (" và khởi động lại robot bringup" if restarted else " (đã lưu cấu hình)")
                })

            # ------------------------------------------------------------------
            # b) POST /api/robot/save-map
            # ------------------------------------------------------------------
            elif self.path == "/api/robot/save-map":
                map_name = data.get("map_name", "").strip()
                if not map_name:
                    map_name = f"map_{int(time.time())}"

                # Normalize base name
                for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                    if map_name.endswith(ext):
                        map_name = map_name[:-len(ext)]

                os.makedirs(MAPS_DIR, exist_ok=True)
                full_base_path = os.path.join(MAPS_DIR, map_name)

                ros_node.get_logger().info(f"Saving map as: {full_base_path}")

                # Step 1: Serialize posegraph (.posegraph and .data) for SLAM Toolbox localization
                ser_ok, ser_msg = ros_node.serialize_map_service(full_base_path, timeout_sec=8.0)
                if not ser_ok:
                    ros_node.get_logger().warn(f"Posegraph serialization warning: {ser_msg}")

                # Step 2: Save 2D occupancy grid (.yaml and .pgm) for Nav2 & visualization
                grid_ok, grid_msg = ros_node.save_map_service(full_base_path, timeout_sec=8.0)
                if not grid_ok:
                    ros_node.get_logger().info("SaveMap service not succeeded, attempting fallback to map_saver_cli...")
                    cmd = [
                        "ros2", "run", "nav2_map_server", "map_saver_cli",
                        "-f", full_base_path
                    ]
                    try:
                        res = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
                        if res.returncode == 0:
                            grid_ok = True
                            grid_msg = "Occupancy grid saved via map_saver_cli"
                        else:
                            grid_msg = res.stderr.strip() or res.stdout.strip() or "map_saver_cli failed"
                    except Exception as e:
                        grid_msg = str(e)

                if ser_ok or grid_ok:
                    set_last_saved_map(map_name)
                    self._send_json({
                        "success": True,
                        "message": f"Bản đồ '{map_name}' đã được lưu thành công",
                        "map_name": map_name,
                        "saved_files": {
                            "posegraph": f"{full_base_path}.posegraph" if os.path.exists(f"{full_base_path}.posegraph") else None,
                            "data": f"{full_base_path}.data" if os.path.exists(f"{full_base_path}.data") else None,
                            "yaml": f"{full_base_path}.yaml" if os.path.exists(f"{full_base_path}.yaml") else None,
                            "pgm": f"{full_base_path}.pgm" if os.path.exists(f"{full_base_path}.pgm") else None
                        }
                    })
                else:
                    self._send_json({
                        "success": False,
                        "message": f"Failed to save map: Serialize ({ser_msg}), Grid ({grid_msg})"
                    }, 500)

            # ------------------------------------------------------------------
            # c) POST /api/robot/home
            # ------------------------------------------------------------------
            elif self.path == "/api/robot/home":
                action = data.get("action", "").strip()

                if action == "set_home":
                    pose = data.get("home_pose") or data.get("pose")
                    if pose and isinstance(pose, list) and len(pose) >= 3:
                        x, y, yaw = round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)
                        save_stored_home_pose([x, y, yaw])
                        self._send_json({
                            "success": True,
                            "home_pose": [x, y, yaw],
                            "message": f"Home pose recorded: x={x}, y={y}, yaw={yaw}"
                        })
                    else:
                        existing = get_stored_home_pose()
                        self._send_json({
                            "success": True,
                            "home_pose": existing,
                            "message": f"Retained existing home pose: x={existing[0]}, y={existing[1]}, yaw={existing[2]}"
                        })

                elif action == "go_home":
                    home_pose = get_stored_home_pose()
                    x, y, yaw = home_pose[0], home_pose[1], home_pose[2]
                    # Trigger Navigation via Behavior Tree
                    ros_node.send_amr_command("GO_HOME")
                    self._send_json({
                        "success": True,
                        "home_pose": [x, y, yaw],
                        "message": f"Dispatched navigation goal to Home pose via Behavior Tree: x={x}, y={y}, yaw={yaw}"
                    })

                else:
                    self._send_json({
                        "success": False,
                        "message": f"Unknown action: '{action}'. Supported actions: 'set_home', 'go_home'"
                    }, 400)

            # ------------------------------------------------------------------
            # d) POST /api/robot/cancel
            # ------------------------------------------------------------------
            elif self.path == "/api/robot/cancel":
                ros_node.send_amr_command("CANCEL")
                self._send_json({
                    "success": True,
                    "message": "Dispatched CANCEL command to Behavior Tree."
                })

            # ------------------------------------------------------------------
            # e) POST /api/robot/bringup
            # ------------------------------------------------------------------
            elif self.path == "/api/robot/bringup":
                action = data.get("action", "").strip().lower()
                ok, msg = control_bringup_service(action)
                self._send_json({
                    "success": ok,
                    "action": action,
                    "message": msg
                }, 200 if ok else 500)

            else:
                self._send_json({"success": False, "message": f"Endpoint '{self.path}' not found"}, 404)

        def log_message(self, format, *args):
            # Suppress normal HTTP access log to keep systemd journal clean
            pass

    return RequestHandler


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    # Initialize ROS 2
    rclpy.init(args=None)
    ros_node = Radxa2AgentNode()

    # Start ROS 2 spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # Create and run HTTP server on 0.0.0.0:5001
    handler_class = make_request_handler(ros_node)
    httpd = ThreadedTCPServer(("0.0.0.0", PORT), handler_class)

    def shutdown_signal(sig, frame):
        ros_node.get_logger().info(f"Signal {sig} received, stopping server...")
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    import signal
    signal.signal(signal.SIGTERM, shutdown_signal)
    signal.signal(signal.SIGINT, shutdown_signal)

    ros_node.get_logger().info(f"Radxa 2 Agent listening on 0.0.0.0:{PORT}")

    try:
        httpd.serve_forever()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        httpd.server_close()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
