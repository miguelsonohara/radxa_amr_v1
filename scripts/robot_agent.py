#!/usr/bin/env python3
"""
robot_agent.py - Local Robot Core Management Daemon
Port: 5001 (127.0.0.1 / 0.0.0.0)
Serves Web Gateway (wifi_manager_node) for:
- Mode switching (SLAM Mapping <-> Nav2 Localization)
- Map scanning, selection, and dual-format saving (PoseGraph + 2D Occupancy Grid)
- Home pose recording and navigation return (publishing /goal_pose)
- Systemd lifecycle management for robot_bringup.service
"""

import http.server
import json
import math
import os
import re
import sys
import socketserver
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, List

# Add scripts directory to sys.path to load ota_engine
sys_scripts_dir = str(Path(__file__).resolve().parent)
if sys_scripts_dir not in sys.path:
    sys.path.insert(0, sys_scripts_dir)

try:
    import ota_engine
    HAS_OTA = True
except Exception as e:
    print(f"[robot_agent] Warning: ota_engine could not be loaded: {e}")
    HAS_OTA = False

try:
    from relocalize_manager import RelocalizeManager
    HAS_RELOCALIZE = True
except Exception as e:
    print(f"[robot_agent] Warning: RelocalizeManager could not be loaded: {e}")
    HAS_RELOCALIZE = False

# ROS 2 imports (graceful fallback if rclpy is not yet sourced)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import BatteryState
    from nav2_msgs.action import DockRobot, UndockRobot
    from tf2_ros import Buffer, TransformListener, TransformException
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

PORT = 5001
WS_DIR = Path(__file__).resolve().parent.parent
MAPS_DIR = Path("/home/rctrobot/robot_data/maps")
PARAMS_FILE = WS_DIR / "src" / "receptionist_robot_bringup" / "config" / "slam_toolbox_params.yaml"
HOME_POSE_FILE = Path("/home/rctrobot/robot_data/home_pose.json")
DOCK_POSE_FILE = Path("/home/rctrobot/robot_data/dock_pose.json")
ANCHORS_FILE = Path("/home/rctrobot/robot_data/anchors.json")
LAST_SAVED_MAP_FILE = Path("/home/rctrobot/robot_data/last_saved_map.txt")
SERVICE_NAME = "robot_bringup.service"


def get_stored_anchors() -> List[Dict[str, Any]]:
    if ANCHORS_FILE.exists():
        try:
            with open(ANCHORS_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return []


def save_stored_anchors(anchors: List[Dict[str, Any]]) -> bool:
    try:
        ANCHORS_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(ANCHORS_FILE, "w", encoding="utf-8") as f:
            json.dump(anchors, f, indent=2, ensure_ascii=False)
        return True
    except Exception:
        return False


# -----------------------------------------------------------------------------
# 1. System & Map Helper Functions
# -----------------------------------------------------------------------------
def run_cmd(cmd, timeout=30) -> Tuple[bool, str, str]:
    """Executes a system shell command with timeout."""
    try:
        res = subprocess.run(
            cmd,
            shell=isinstance(cmd, str),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return res.returncode == 0, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def get_stored_home_pose() -> List[float]:
    """Retrieves recorded home pose [x, y, yaw] from disk."""
    if HOME_POSE_FILE.exists():
        try:
            with open(HOME_POSE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                if isinstance(data, list) and len(data) >= 3:
                    return [round(float(data[0]), 3), round(float(data[1]), 3), round(float(data[2]), 3)]
        except Exception as e:
            print(f"[robot_agent] Error reading home pose: {e}")
    return [0.0, 0.0, 0.0]


def save_stored_home_pose(pose: List[float]):
    """Persists recorded home pose [x, y, yaw] to disk."""
    try:
        HOME_POSE_FILE.parent.mkdir(parents=True, exist_ok=True)
        clean_pose = [round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)]
        with open(HOME_POSE_FILE, "w", encoding="utf-8") as f:
            json.dump(clean_pose, f, indent=2)
        print(f"[robot_agent] Saved home pose: {clean_pose}")
    except Exception as e:
        print(f"[robot_agent] Error saving home pose: {e}")


def get_stored_dock_pose() -> Optional[List[float]]:
    """Retrieves recorded dock pose [x, y, yaw] in map frame, or None if not set."""
    if DOCK_POSE_FILE.exists():
        try:
            with open(DOCK_POSE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                if isinstance(data, list) and len(data) >= 3:
                    return [round(float(data[0]), 4), round(float(data[1]), 4), round(float(data[2]), 4)]
        except Exception as e:
            print(f"[robot_agent] Error reading dock pose: {e}")
    return None


def save_stored_dock_pose(pose: List[float]):
    """Persists recorded dock pose [x, y, yaw] to disk."""
    try:
        DOCK_POSE_FILE.parent.mkdir(parents=True, exist_ok=True)
        clean_pose = [round(float(pose[0]), 4), round(float(pose[1]), 4), round(float(pose[2]), 4)]
        with open(DOCK_POSE_FILE, "w", encoding="utf-8") as f:
            json.dump(clean_pose, f, indent=2)
        print(f"[robot_agent] Saved dock pose: {clean_pose}")
    except Exception as e:
        print(f"[robot_agent] Error saving dock pose: {e}")


def get_last_saved_map() -> str:
    """Gets the name of the most recently saved or loaded map."""
    if LAST_SAVED_MAP_FILE.exists():
        try:
            with open(LAST_SAVED_MAP_FILE, "r", encoding="utf-8") as f:
                return f.read().strip()
        except Exception:
            pass
    return ""


def set_last_saved_map(name: str):
    """Records the active/last saved map base name."""
    try:
        clean_name = os.path.basename(name).strip()
        for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
            if clean_name.endswith(ext):
                clean_name = clean_name[:-len(ext)]
        with open(LAST_SAVED_MAP_FILE, "w", encoding="utf-8") as f:
            f.write(clean_name)
    except Exception as e:
        print(f"[robot_agent] Error saving last_saved_map: {e}")


def read_slam_params() -> Tuple[str, str]:
    """Reads mode and map_file_name from slam_toolbox_params.yaml."""
    mode = "mapping"
    map_file_name = ""
    if not PARAMS_FILE.exists():
        return mode, map_file_name

    try:
        with open(PARAMS_FILE, "r", encoding="utf-8") as f:
            content = f.read()

        mode_match = re.search(r"^[ \t]*mode:[ \t]*([^\r\n#]+)", content, flags=re.MULTILINE)
        if mode_match:
            mode = mode_match.group(1).strip()

        map_match = re.search(r"^[ \t]*map_file_name:[ \t]*([^\r\n#]+)", content, flags=re.MULTILINE)
        if map_match:
            map_file_name = map_match.group(1).strip()
    except Exception as e:
        print(f"[robot_agent] Error reading slam params: {e}")

    return mode, map_file_name


def update_slam_params(mode: str, map_file_name: Optional[str] = None):
    """Updates slam_toolbox_params.yaml for mapping or localization."""
    if not PARAMS_FILE.exists():
        print(f"[robot_agent] Warning: {PARAMS_FILE} does not exist!")
        return

    try:
        with open(PARAMS_FILE, "r", encoding="utf-8") as f:
            content = f.read()

        # 1. Update mode line
        pattern_mode = r"(^[ \t]*mode:[ \t]*)[^\r\n#]+"
        if re.search(pattern_mode, content, flags=re.MULTILINE):
            content = re.sub(pattern_mode, r"\g<1>" + mode, content, flags=re.MULTILINE)

        # 2. Handle map_file_name based on mode
        if mode == "mapping":
            # Comment out map_file_name so SLAM starts with clean blank canvas
            pattern_map = r"(^[ \t]*)(map_file_name:[ \t]*[^\r\n#]+)"
            content = re.sub(pattern_map, r"\g<1># \g<2>", content, flags=re.MULTILINE)
        elif mode == "localization" and map_file_name:
            pattern_map_active = r"(^[ \t]*)(map_file_name:[ \t]*)[^\r\n#]+"
            pattern_map_commented = r"(^[ \t]*)#[ \t]*(map_file_name:[ \t]*)[^\r\n#]+"
            if re.search(pattern_map_active, content, flags=re.MULTILINE):
                content = re.sub(pattern_map_active, r"\g<1>\g<2>" + map_file_name, content, flags=re.MULTILINE)
            elif re.search(pattern_map_commented, content, flags=re.MULTILINE):
                content = re.sub(pattern_map_commented, r"\g<1>\g<2>" + map_file_name, content, flags=re.MULTILINE)
            else:
                content = re.sub(r"(^[ \t]*mode:[ \t]*[^\r\n]+)", r"\1\n    map_file_name: " + map_file_name, content, flags=re.MULTILINE)

        with open(PARAMS_FILE, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"[robot_agent] Updated slam params: mode={mode}, map={map_file_name}")
    except Exception as e:
        print(f"[robot_agent] Error updating slam params: {e}")


def scan_available_maps() -> List[Dict[str, Any]]:
    """Scans MAPS_DIR and returns sorted metadata of available maps."""
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    maps_dict = {}

    for fpath in MAPS_DIR.iterdir():
        if not fpath.is_file():
            continue
        base = fpath.stem
        ext = fpath.suffix.lower()
        if ext not in [".posegraph", ".data", ".yaml", ".pgm"]:
            continue
        if base in ["keepout_mask"]:
            continue

        if base not in maps_dict:
            mtime = fpath.stat().st_mtime
            maps_dict[base] = {
                "name": base,
                "full_path": str(MAPS_DIR / base),
                "has_posegraph": False,
                "has_yaml": False,
                "modified": mtime,
                "modified_str": time.strftime("%d/%m/%Y %H:%M", time.localtime(mtime)),
                "total_bytes": 0,
            }

        m = maps_dict[base]
        if ext == ".posegraph":
            m["has_posegraph"] = True
        elif ext == ".yaml":
            m["has_yaml"] = True
        m["total_bytes"] += fpath.stat().st_size
        f_mtime = fpath.stat().st_mtime
        if f_mtime > m["modified"]:
            m["modified"] = f_mtime
            m["modified_str"] = time.strftime("%d/%m/%Y %H:%M", time.localtime(f_mtime))

    map_list = list(maps_dict.values())
    for m in map_list:
        m["size_mb"] = round(m["total_bytes"] / (1024 * 1024), 2)

    map_list.sort(key=lambda x: x["modified"], reverse=True)
    return map_list


def is_service_active(service_name: str = SERVICE_NAME) -> bool:
    """Checks if systemd service is active."""
    ok, out, _ = run_cmd(f"systemctl is-active {service_name}")
    return ok and out.strip() == "active"


def control_bringup_service(action: str) -> Tuple[bool, str]:
    """Starts, stops, or restarts robot_bringup.service."""
    action = action.strip().lower()
    if action not in ["start", "stop", "restart"]:
        return False, f"Hành động không hợp lệ: {action}"

    cmd = ["sudo", "-n", "systemctl", action, SERVICE_NAME]
    ok, _, err = run_cmd(cmd, timeout=15)
    if not ok and ("password" in err.lower() or "privilege" in err.lower()):
        # Try direct systemctl if user has polkit authorization
        ok, _, err = run_cmd(["systemctl", action, SERVICE_NAME], timeout=15)

    if ok:
        return True, f"Đã thực hiện {action} dịch vụ {SERVICE_NAME}"
    else:
        return False, err or f"Lỗi {action} {SERVICE_NAME}"


# -----------------------------------------------------------------------------
# 2. ROS 2 Helper Node (Publishing /goal_pose)
# -----------------------------------------------------------------------------
class RobotAgentROSNode:
    def __init__(self):
        self.node = None
        self.goal_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.battery_sub = None
        self.dock_client = None
        self.undock_client = None
        self.battery_lock = threading.Lock()
        self.latest_battery: Optional[Dict[str, Any]] = None
        self.spin_thread = None

        if HAS_RCLPY:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = Node("robot_agent_core")
                self.goal_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self.node)
                self.battery_sub = self.node.create_subscription(
                    BatteryState, "/battery_state", self._battery_cb, 10
                )
                self.dock_client = ActionClient(self.node, DockRobot, "/dock_robot")
                self.undock_client = ActionClient(self.node, UndockRobot, "/undock_robot")
                self.spin_thread = threading.Thread(target=self._spin, daemon=True)
                self.spin_thread.start()
                print("[robot_agent] ROS 2 core node initialized successfully")
            except Exception as e:
                print(f"[robot_agent] Warning: Failed to init rclpy: {e}")

    def _battery_cb(self, msg):
        with self.battery_lock:
            self.latest_battery = {
                "voltage": round(float(msg.voltage), 2),
                "current": round(float(msg.current), 2),
                "percentage": round(float(msg.percentage), 3),
                "power_supply_status": int(msg.power_supply_status),
                "is_charging": int(msg.power_supply_status) == 1 or float(msg.current) > 0.5,
                "timestamp": time.time(),
            }

    def get_battery_status(self) -> Dict[str, Any]:
        with self.battery_lock:
            if self.latest_battery is None:
                return {"available": False, "message": "Chưa nhận được dữ liệu từ /battery_state"}
            return {"available": True, **self.latest_battery}

    def get_current_map_pose(self) -> Optional[List[float]]:
        """Reads current robot pose [x, y, yaw] in map frame via TF2."""
        if not (self.node and self.tf_buffer):
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return [round(x, 4), round(y, 4), round(yaw, 4)]
        except TransformException as e:
            print(f"[robot_agent] TF lookup map->base_link failed: {e}")
            return None

    def send_dock_goal(self, x: float, y: float, yaw: float) -> Tuple[bool, str]:
        """Sends DockRobot action goal using dynamic dock_pose (use_dock_id=False)."""
        if not (self.node and self.dock_client):
            return False, "ROS 2 DockRobot ActionClient chưa sẵn sàng"
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            return False, "/dock_robot action server chưa hoạt động (kiểm tra docking_server)"

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_type = "simple_charging_dock"
        goal.navigate_to_staging_pose = True
        goal.dock_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.dock_pose.header.frame_id = "map"
        goal.dock_pose.pose.position.x = float(x)
        goal.dock_pose.pose.position.y = float(y)
        goal.dock_pose.pose.position.z = 0.0
        goal.dock_pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.dock_pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.dock_client.send_goal_async(goal)
        return True, f"Đã gửi lệnh DockRobot tới [{x:.3f}, {y:.3f}, {yaw:.3f}]"

    def send_undock_goal(self) -> Tuple[bool, str]:
        """Sends UndockRobot action goal."""
        if not (self.node and self.undock_client):
            return False, "ROS 2 UndockRobot ActionClient chưa sẵn sàng"
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            return False, "/undock_robot action server chưa hoạt động"

        goal = UndockRobot.Goal()
        goal.dock_type = "simple_charging_dock"
        goal.max_undocking_time = 30.0
        self.undock_client.send_goal_async(goal)
        return True, "Đã gửi lệnh UndockRobot lùi ra khỏi trạm sạc"

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    def publish_goal(self, x: float, y: float, yaw: float) -> bool:
        """Publishes goal pose to Nav2 /goal_pose topic."""
        # Convert yaw (rad) to quaternion around Z
        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        if self.node and self.goal_pub:
            try:
                msg = PoseStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.position.x = float(x)
                msg.pose.position.y = float(y)
                msg.pose.position.z = 0.0
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw
                self.goal_pub.publish(msg)
                print(f"[robot_agent] Published /goal_pose via rclpy: x={x}, y={y}, yaw={yaw}")
                return True
            except Exception as e:
                print(f"[robot_agent] rclpy publish failed: {e}")

        # Fallback: publish via CLI ros2 topic pub
        goal_yaml = json.dumps({
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": float(qz), "w": float(qw)}
            }
        })
        cli_cmd = f"ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{goal_yaml}'"
        ok, _, err = run_cmd(cli_cmd, timeout=5)
        if ok:
            print(f"[robot_agent] Published /goal_pose via CLI fallback: x={x}, y={y}")
            return True
        else:
            print(f"[robot_agent] CLI goal publish error: {err}")
            return False


# Global instance of ROS node and Relocalize Manager
ros_core: Optional[RobotAgentROSNode] = None
relocalize_mgr: Optional[Any] = None


# -----------------------------------------------------------------------------
# 3. HTTP Request Handler
# -----------------------------------------------------------------------------
class RobotAgentHTTPHandler(http.server.BaseHTTPRequestHandler):
    def _send_json(self, data, code=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]

        # ----------------------------------------------------------------------
        # a) GET /api/robot/status
        # ----------------------------------------------------------------------
        if path == "/api/robot/status":
            mode, current_map = read_slam_params()
            home_pose = get_stored_home_pose()
            bringup_active = is_service_active(SERVICE_NAME)

            # Check if SLAM or Nav2 nodes are visible (in-memory via rclpy, no CLI subprocess)
            if ros_core and ros_core.node:
                try:
                    node_names = ros_core.node.get_node_names()
                    slam_active = any("slam_toolbox" in n for n in node_names)
                except Exception:
                    slam_active = bringup_active
            else:
                ok_nodes, out_nodes, _ = run_cmd("ros2 node list 2>/dev/null", timeout=3)
                slam_active = ok_nodes and ("slam_toolbox" in out_nodes or "async_slam_toolbox_node" in out_nodes)

            self._send_json({
                "mode": mode,
                "home_pose": home_pose,
                "current_map": current_map,
                "current_map_name": os.path.basename(current_map) if current_map else "",
                "last_saved_map": get_last_saved_map(),
                "slam_active": slam_active or bringup_active,
                "bringup_service_active": bringup_active,
            })

        # ----------------------------------------------------------------------
        # b) GET /api/robot/maps
        # ----------------------------------------------------------------------
        elif path == "/api/robot/maps":
            _, current_map = read_slam_params()
            current_base = os.path.basename(current_map) if current_map else ""
            maps = scan_available_maps()
            self._send_json({
                "success": True,
                "current_map": current_base,
                "current_map_path": current_map,
                "last_saved_map": get_last_saved_map(),
                "maps": maps,
            })

        # ----------------------------------------------------------------------
        # c) GET /api/ota/status
        # ----------------------------------------------------------------------
        elif path == "/api/ota/status":
            if HAS_OTA:
                self._send_json(ota_engine.engine.get_status())
            else:
                self._send_json({"success": False, "message": "OTA Engine không khả dụng"}, 500)

        # ----------------------------------------------------------------------
        # d) GET /api/robot/relocalize/status
        # ----------------------------------------------------------------------
        elif path == "/api/robot/relocalize/status":
            if relocalize_mgr:
                self._send_json(relocalize_mgr.get_status())
            else:
                self._send_json({
                    "state": "idle",
                    "progress": 0,
                    "message": "Relocalize Manager chưa sẵn sàng",
                    "is_running": False
                })

        # ----------------------------------------------------------------------
        # e) GET /api/robot/anchors
        # ----------------------------------------------------------------------
        elif path == "/api/robot/anchors":
            anchors = get_stored_anchors()
            self._send_json({"success": True, "anchors": anchors})

        # ----------------------------------------------------------------------
        # f) GET /api/robot/dock-pose & GET /api/robot/battery
        # ----------------------------------------------------------------------
        elif path == "/api/robot/dock-pose":
            dock_pose = get_stored_dock_pose()
            self._send_json({
                "success": True,
                "configured": dock_pose is not None,
                "dock_pose": dock_pose,
            })

        elif path == "/api/robot/battery":
            if ros_core:
                self._send_json({"success": True, **ros_core.get_battery_status()})
            else:
                self._send_json({"success": False, "available": False, "message": "ROS node chưa khởi tạo"}, 500)

        else:
            self._send_json({"success": False, "message": f"Endpoint '{path}' not found"}, 404)

    def do_POST(self):
        global ros_core, relocalize_mgr
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len) if content_len > 0 else b"{}"

        try:
            data = json.loads(body.decode("utf-8"))
        except Exception:
            data = {}

        path = self.path.split("?")[0]

        # ----------------------------------------------------------------------
        # 1. POST /api/robot/mode (Switch SLAM Mapping <-> Nav2 Localization)
        # ----------------------------------------------------------------------
        if path == "/api/robot/mode":
            requested_mode = data.get("mode", "").strip().lower()

            if requested_mode == "mapping":
                update_slam_params("mapping")
                save_stored_home_pose([0.0, 0.0, 0.0])
                ok_restart, msg_restart = control_bringup_service("restart")
                self._send_json({
                    "success": ok_restart,
                    "mode": "mapping",
                    "home_pose": [0.0, 0.0, 0.0],
                    "message": f"Đã chuyển sang chế độ Quét Bản Đồ (SLAM). {msg_restart}",
                })

            elif requested_mode == "localization":
                map_file_name = data.get("map_file_name", data.get("map_name", "")).strip()
                if not map_file_name:
                    last_saved = get_last_saved_map()
                    if last_saved:
                        map_file_name = str(MAPS_DIR / last_saved)
                    else:
                        _, existing_map = read_slam_params()
                        map_file_name = existing_map or str(MAPS_DIR / "demo_map1")

                # Strip file extensions
                base_map = map_file_name
                for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                    if base_map.endswith(ext):
                        base_map = base_map[:-len(ext)]

                if not os.path.isabs(base_map):
                    base_map = str(MAPS_DIR / base_map)

                clean_name = os.path.basename(base_map)
                set_last_saved_map(clean_name)
                update_slam_params("localization", base_map)
                home_pose = get_stored_home_pose()
                ok_restart, msg_restart = control_bringup_service("restart")

                self._send_json({
                    "success": ok_restart,
                    "mode": "localization",
                    "home_pose": home_pose,
                    "map_name": clean_name,
                    "map_file_name": base_map,
                    "message": f"Đã chuyển sang chế độ Vận Hành với bản đồ '{clean_name}'. {msg_restart}",
                })
            else:
                self._send_json({"success": False, "message": f"Chế độ không hợp lệ: {requested_mode}"}, 400)

        # ----------------------------------------------------------------------
        # 2. POST /api/robot/select-map
        # ----------------------------------------------------------------------
        elif path == "/api/robot/select-map":
            map_name = data.get("map_name", "").strip()
            if not map_name:
                self._send_json({"success": False, "message": "map_name is required"}, 400)
                return

            base_map = map_name
            for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                if base_map.endswith(ext):
                    base_map = base_map[:-len(ext)]

            full_map_path = str(MAPS_DIR / base_map)
            set_last_saved_map(base_map)
            update_slam_params("localization", full_map_path)
            ok_restart, msg_restart = control_bringup_service("restart")

            self._send_json({
                "success": ok_restart,
                "map_name": base_map,
                "map_file_name": full_map_path,
                "message": f"Đã chọn nạp bản đồ '{base_map}' vào chế độ Vận Hành. {msg_restart}",
            })

        # ----------------------------------------------------------------------
        # 3. POST /api/robot/save-map (Dual Save: PoseGraph + 2D Yaml/Pgm)
        # ----------------------------------------------------------------------
        elif path == "/api/robot/save-map":
            raw_map_name = data.get("map_name", "").strip()
            if not raw_map_name:
                raw_map_name = f"map_{time.strftime('%Y%m%d_%H%M%S')}"

            # Strip extension
            map_name = raw_map_name
            for ext in [".posegraph", ".data", ".yaml", ".pgm"]:
                if map_name.endswith(ext):
                    map_name = map_name[:-len(ext)]

            MAPS_DIR.mkdir(parents=True, exist_ok=True)
            save_path = str(MAPS_DIR / map_name)

            # 1. Call SLAM Toolbox Serialize Map Service
            ser_cmd = (
                f"ros2 service call /slam_toolbox/serialize_map "
                f"slam_toolbox/srv/SerializePoseGraph '{{filename: \"{save_path}\"}}'"
            )
            ser_ok, ser_out, ser_err = run_cmd(ser_cmd, timeout=12)

            # 2. Call Nav2 map_saver_cli
            grid_cmd = f"ros2 run nav2_map_server map_saver_cli -f '{save_path}'"
            grid_ok, grid_out, grid_err = run_cmd(grid_cmd, timeout=12)

            if ser_ok or grid_ok:
                set_last_saved_map(map_name)
                self._send_json({
                    "success": True,
                    "message": f"Bản đồ '{map_name}' đã được lưu thành công",
                    "map_name": map_name,
                    "posegraph_saved": ser_ok,
                    "occupancy_grid_saved": grid_ok,
                })
            else:
                self._send_json({
                    "success": False,
                    "message": f"Lỗi khi lưu bản đồ: {ser_err or grid_err or 'Không thể kết nối SLAM node'}",
                }, 500)

        # ----------------------------------------------------------------------
        # 4. POST /api/robot/home (set_home / go_home)
        # ----------------------------------------------------------------------
        elif path == "/api/robot/home":
            action = data.get("action", "").strip().lower()

            if action == "set_home":
                pose = data.get("home_pose") or data.get("pose")
                if pose and isinstance(pose, list) and len(pose) >= 3:
                    x, y, yaw = round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)
                    save_stored_home_pose([x, y, yaw])
                    self._send_json({
                        "success": True,
                        "home_pose": [x, y, yaw],
                        "message": f"Đã ghi nhớ vị trí Home mới: x={x}, y={y}, yaw={yaw}",
                    })
                else:
                    existing = get_stored_home_pose()
                    self._send_json({
                        "success": True,
                        "home_pose": existing,
                        "message": f"Đã giữ nguyên vị trí Home hiện tại: {existing}",
                    })

            elif action == "go_home":
                target_pose = data.get("home_pose") or get_stored_home_pose()
                x, y, yaw = target_pose[0], target_pose[1], target_pose[2]

                global ros_core
                pub_ok = False
                if ros_core:
                    pub_ok = ros_core.publish_goal(x, y, yaw)
                else:
                    # Retry instantiate
                    ros_core = RobotAgentROSNode()
                    pub_ok = ros_core.publish_goal(x, y, yaw)

                if pub_ok:
                    self._send_json({
                        "success": True,
                        "target_pose": [x, y, yaw],
                        "message": f"Đã phát lệnh dẫn đường quay về Home: x={x}, y={y}, yaw={yaw}",
                    })
                else:
                    self._send_json({
                        "success": False,
                        "message": "Không thể gửi mục tiêu tới /goal_pose. Vui lòng kiểm tra Nav2 stack!",
                    }, 500)
            else:
                self._send_json({"success": False, "message": f"Hành động không hợp lệ: {action}"}, 400)

        # ----------------------------------------------------------------------
        # 4b. POST /api/robot/dock-pose, /api/robot/dock, /api/robot/undock
        # ----------------------------------------------------------------------
        elif path == "/api/robot/dock-pose":
            pose = data.get("dock_pose") or data.get("pose")
            if isinstance(pose, list) and len(pose) >= 3:
                saved = [float(pose[0]), float(pose[1]), float(pose[2])]
            else:
                saved = ros_core.get_current_map_pose() if ros_core else None
                if saved is None:
                    self._send_json({
                        "success": False,
                        "message": "Không đọc được TF map -> base_link để lưu tọa độ dock!",
                    }, 500)
                    return
            save_stored_dock_pose(saved)
            self._send_json({
                "success": True,
                "dock_pose": get_stored_dock_pose(),
                "message": f"Đã lưu tọa độ trạm sạc: {saved}",
            })

        elif path == "/api/robot/dock":
            dock_pose = get_stored_dock_pose()
            if not dock_pose:
                self._send_json({
                    "success": False,
                    "message": "Chưa lưu tọa độ trạm sạc! Hãy lái xe vào dock và gọi POST /api/robot/dock-pose trước.",
                }, 400)
                return
            ok, msg = ros_core.send_dock_goal(dock_pose[0], dock_pose[1], dock_pose[2]) if ros_core else (False, "ROS chưa sẵn sàng")
            self._send_json({"success": ok, "dock_pose": dock_pose, "message": msg}, 200 if ok else 500)

        elif path == "/api/robot/undock":
            ok, msg = ros_core.send_undock_goal() if ros_core else (False, "ROS chưa sẵn sàng")
            self._send_json({"success": ok, "message": msg}, 200 if ok else 500)

        # ----------------------------------------------------------------------
        # 5. POST /api/robot/bringup (Service start/stop/restart)
        # ----------------------------------------------------------------------
        elif path == "/api/robot/bringup":
            action = data.get("action", "restart").strip().lower()
            ok, msg = control_bringup_service(action)
            time.sleep(0.5)
            active = is_service_active(SERVICE_NAME)
            self._send_json({
                "success": ok,
                "action": action,
                "bringup_service_active": active,
                "message": msg,
            }, code=200 if ok else 500)

        # ----------------------------------------------------------------------
        # 6. POST /api/connect-wifi (Backward compatibility)
        # ----------------------------------------------------------------------
        elif path == "/api/connect-wifi":
            ssid = data.get("ssid", "").strip()
            password = data.get("password", "").strip()
            if not ssid:
                self._send_json({"success": False, "message": "SSID is required"}, 400)
                return

            cmd = ["nmcli", "dev", "wifi", "connect", ssid]
            if password:
                cmd.extend(["password", password])

            ok, out, err = run_cmd(cmd, timeout=30)
            if ok:
                self._send_json({"success": True, "message": f"Connected to {ssid}"})
            else:
                self._send_json({"success": False, "message": err or out or "Failed to connect"}, 500)

        # ----------------------------------------------------------------------
        # 7. POST /api/ota/check (Check GitHub Releases for updates)
        # ----------------------------------------------------------------------
        elif path == "/api/ota/check":
            if not HAS_OTA:
                self._send_json({"success": False, "message": "OTA Engine không khả dụng"}, 500)
                return
            res = ota_engine.engine.check_for_updates()
            self._send_json(res, code=200 if res.get("success") else 500)

        # ----------------------------------------------------------------------
        # 8. POST /api/ota/apply (Trigger full asynchronous OTA update)
        # ----------------------------------------------------------------------
        elif path == "/api/ota/apply":
            if not HAS_OTA:
                self._send_json({"success": False, "message": "OTA Engine không khả dụng"}, 500)
                return
            
            status = ota_engine.engine.get_status()
            if status.get("is_busy"):
                self._send_json({
                    "success": False,
                    "message": f"Hệ thống đang bận cập nhật ở tiến trình {status.get('stage')} ({status.get('progress')}%)",
                }, code=409)
                return

            download_url = data.get("download_url", "").strip()
            version = data.get("version", "").strip()
            sha256 = data.get("sha256", "").strip()

            if not download_url:
                chk = ota_engine.engine.check_for_updates()
                if chk.get("success") and chk.get("download_url"):
                    download_url = chk["download_url"]
                    version = version or chk.get("latest_version", "")
                    sha256 = sha256 or chk.get("sha256", "")
                    data["download_url"] = download_url
                    data["version"] = version
                    data["sha256"] = sha256
                else:
                    self._send_json({"success": False, "message": "download_url là bắt buộc và không tìm thấy bản phát hành mới trên máy chủ"}, 400)
                    return

            # Launch background OTA update thread
            threading.Thread(
                target=ota_engine.engine.run_full_update,
                args=(data,),
                daemon=True
            ).start()

            self._send_json({
                "success": True,
                "stage": "CHECKING_SAFETY",
                "progress": 5,
                "message": f"Đã bắt đầu tiến trình cập nhật lên {version}...",
            })

        # ----------------------------------------------------------------------
        # 9. POST /api/ota/rollback (Manual trigger to rollback)
        # ----------------------------------------------------------------------
        elif path == "/api/ota/rollback":
            if not HAS_OTA:
                self._send_json({"success": False, "message": "OTA Engine không khả dụng"}, 500)
                return

            status = ota_engine.engine.get_status()
            if status.get("is_busy"):
                self._send_json({
                    "success": False,
                    "message": "Không thể rollback khi đang trong tiến trình cập nhật!",
                }, code=409)
                return

            ok, msg = ota_engine.engine.rollback()
            self._send_json({"success": ok, "message": msg}, code=200 if ok else 500)

        # ----------------------------------------------------------------------
        # 10. POST /api/robot/relocalize (Autonomous 360 Spin / Pose Reset)
        # ----------------------------------------------------------------------
        elif path == "/api/robot/relocalize":
            action = data.get("action", "start").strip().lower()

            # Ensure relocalize_mgr exists
            if not relocalize_mgr and HAS_RELOCALIZE:
                if not ros_core:
                    ros_core = RobotAgentROSNode()
                if ros_core and ros_core.node:
                    relocalize_mgr = RelocalizeManager(ros_core.node)

            if not relocalize_mgr:
                self._send_json({"success": False, "message": "Relocalize Manager không khả dụng (ROS node chưa kết nối)"}, 500)
                return

            if action == "start":
                res = relocalize_mgr.start_relocalization()
                self._send_json(res, code=200 if res.get("success") else 400)
            elif action == "stop":
                res = relocalize_mgr.stop_relocalization()
                self._send_json(res)
            elif action in ["set_pose", "set_initial_pose"]:
                pose = data.get("pose")
                if pose and isinstance(pose, list) and len(pose) >= 3:
                    ok = relocalize_mgr.set_initial_pose(pose[0], pose[1], pose[2])
                    self._send_json({
                        "success": ok,
                        "pose": pose,
                        "message": f"Đã phát /set_pose tại ({pose[0]}, {pose[1]}, {pose[2]})",
                    }, code=200 if ok else 500)
                else:
                    self._send_json({"success": False, "message": "pose [x, y, yaw] là bắt buộc"}, 400)
            else:
                self._send_json({"success": False, "message": f"Hành động không hợp lệ: {action}"}, 400)

        # ----------------------------------------------------------------------
        # 11. POST /api/robot/anchors (Save/Update list of known anchor points)
        # ----------------------------------------------------------------------
        elif path == "/api/robot/anchors":
            anchors = data.get("anchors")
            if isinstance(anchors, list):
                ok = save_stored_anchors(anchors)
                self._send_json({
                    "success": ok,
                    "anchors": anchors,
                    "message": "Đã lưu danh sách anchor points thành công",
                })
            else:
                self._send_json({"success": False, "message": "anchors list là bắt buộc"}, 400)

        else:
            self._send_json({"success": False, "message": f"Endpoint '{path}' not found"}, 404)

    def log_message(self, format, *args):
        # Keep terminal log clean
        pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def main():
    global ros_core, relocalize_mgr
    ros_core = RobotAgentROSNode()
    if HAS_RELOCALIZE and ros_core and ros_core.node:
        try:
            relocalize_mgr = RelocalizeManager(ros_core.node)
        except Exception as e:
            print(f"[robot_agent] Warning: Failed to init RelocalizeManager: {e}")

    server_address = ("0.0.0.0", PORT)
    print(f"[robot_agent] Starting Robot Core Agent Daemon on http://0.0.0.0:{PORT}...")
    with ThreadedTCPServer(server_address, RobotAgentHTTPHandler) as httpd:
        print(f"[robot_agent] Ready to receive commands from Web Gateway (wifi_manager_node).")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[robot_agent] Shutting down.")


if __name__ == "__main__":
    main()
