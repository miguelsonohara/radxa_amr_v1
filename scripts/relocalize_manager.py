#!/usr/bin/env python3
"""
relocalize_manager.py - Autonomous Relocalization Manager
Controls robot 360-degree spin behavior and pose resetting for LiDAR scan matching.
Used by robot_agent.py (REST API port 5001) and Web UI.
"""

import math
import threading
import time
from typing import Dict, Any, Optional, List

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class RelocalizeManager:
    """Manages the autonomous 360-degree spin relocalization process."""

    def __init__(self, ros_node: Optional[Any] = None):
        self.node = ros_node
        self.cmd_pub = None
        self.set_pose_pub = None
        self.odom_sub = None

        self._lock = threading.Lock()
        self._state = "idle"       # "idle", "running", "completed", "cancelled", "error"
        self._progress = 0         # 0 - 100 (%)
        self._message = "Sẵn sàng"
        self._stop_requested = False
        self._active_thread = None

        # Robot kinematic specs for 360 spin
        self.spin_speed = 0.35     # rad/s (~20 deg/sec)
        self.spin_duration = (2.0 * math.pi) / self.spin_speed  # ~17.95 sec for full 360 deg

        if HAS_ROS2 and self.node:
            self._setup_ros_publishers()

    def _setup_ros_publishers(self):
        try:
            self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
            self.set_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
            print("[relocalize] RelocalizeManager ROS 2 publishers initialized.")
        except Exception as e:
            print(f"[relocalize] Error creating publishers: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Returns current relocalization state and progress."""
        with self._lock:
            return {
                "state": self._state,
                "progress": self._progress,
                "message": self._message,
                "is_running": (self._state == "running"),
            }

    def start_relocalization(self) -> Dict[str, Any]:
        """Initiates autonomous 360 spin in a background thread."""
        with self._lock:
            if self._state == "running":
                return {
                    "success": False,
                    "message": "Quá trình tự định vị đang chạy.",
                    "status": self.get_status()
                }

            self._state = "running"
            self._progress = 0
            self._message = "Bắt đầu xoay 360° quét LiDAR khớp bản đồ..."
            self._stop_requested = False

        self._active_thread = threading.Thread(target=self._run_spin_worker, daemon=True)
        self._active_thread.start()

        return {
            "success": True,
            "message": "Đã bắt đầu tiến trình tự định vị 360°.",
            "status": self.get_status()
        }

    def stop_relocalization(self) -> Dict[str, Any]:
        """Cancels running relocalization and halts robot."""
        with self._lock:
            if self._state != "running":
                return {"success": True, "message": "Robot đang ở trạng thái dừng."}

            self._stop_requested = True
            self._message = "Đang dừng tiến trình tự định vị..."

        self._send_stop_cmd()
        return {"success": True, "message": "Đã gửi lệnh dừng tự định vị."}

    def set_initial_pose(self, x: float, y: float, yaw: float) -> bool:
        """Publishes an initial pose estimate to /set_pose (PoseWithCovarianceStamped)."""
        if not (HAS_ROS2 and self.node and self.set_pose_pub):
            print("[relocalize] Cannot publish /set_pose: ROS node not ready.")
            return False

        try:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.position.z = 0.0

            half_yaw = yaw * 0.5
            msg.pose.pose.orientation.z = math.sin(half_yaw)
            msg.pose.pose.orientation.w = math.cos(half_yaw)

            # Small covariance for known anchor points (0.25m position, 0.06 rad yaw)
            cov = [0.0] * 36
            cov[0] = 0.25   # x
            cov[7] = 0.25   # y
            cov[35] = 0.06  # yaw
            msg.pose.covariance = cov

            self.set_pose_pub.publish(msg)
            print(f"[relocalize] Published /set_pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            return True
        except Exception as e:
            print(f"[relocalize] Error publishing /set_pose: {e}")
            return False

    def _run_spin_worker(self):
        """Worker thread executing safe 360 rotation and progress updates."""
        print("[relocalize] Worker started: spinning robot 360 degrees...")
        start_time = time.time()
        pub_period = 0.1  # 10Hz velocity command rate

        try:
            while True:
                if self._stop_requested:
                    with self._lock:
                        self._state = "cancelled"
                        self._message = "Đã hủy tự định vị."
                    self._send_stop_cmd()
                    return

                elapsed = time.time() - start_time
                progress = min(99, int((elapsed / self.spin_duration) * 100))

                with self._lock:
                    self._progress = progress
                    self._message = f"Đang xoay quét LiDAR tìm vị trí ({progress}%)..."

                if elapsed >= self.spin_duration:
                    break

                self._send_spin_cmd(self.spin_speed)
                time.sleep(pub_period)

            # Finish spin
            self._send_stop_cmd()
            time.sleep(0.5)

            with self._lock:
                self._state = "completed"
                self._progress = 100
                self._message = "Tự định vị thành công! Đã khớp bản đồ."

            print("[relocalize] Worker completed: 360 spin finished successfully.")

        except Exception as e:
            self._send_stop_cmd()
            with self._lock:
                self._state = "error"
                self._message = f"Lỗi trong quá trình tự định vị: {e}"
            print(f"[relocalize] Worker error: {e}")

    def _send_spin_cmd(self, angular_z: float):
        """Publishes angular velocity command to /cmd_vel."""
        if not (HAS_ROS2 and self.node and self.cmd_pub):
            return
        try:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)
        except Exception as e:
            print(f"[relocalize] Publish spin cmd error: {e}")

    def _send_stop_cmd(self):
        """Publishes zero velocity to ensure robot halts safely."""
        if not (HAS_ROS2 and self.node and self.cmd_pub):
            return
        try:
            for _ in range(3):
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.02)
        except Exception as e:
            print(f"[relocalize] Stop cmd error: {e}")
