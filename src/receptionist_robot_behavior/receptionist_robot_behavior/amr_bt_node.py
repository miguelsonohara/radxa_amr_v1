#!/usr/bin/env python3
"""
amr_bt_node.py - AMR Behavior Tree Central Orchestrator Node
"""

import json
import os
import time
import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from receptionist_robot_behavior.tree import create_amr_behavior_tree


HOME_POSE_FILE = "/home/radxa/home_pose.json"


def read_stored_home():
    if os.path.exists(HOME_POSE_FILE):
        try:
            with open(HOME_POSE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                if "home_pose" in data and isinstance(data["home_pose"], list):
                    return [float(x) for x in data["home_pose"]]
                if "x" in data and "y" in data:
                    return [float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("yaw", 0.0))]
        except Exception:
            pass
    return [0.0, 0.0, 0.0]


class AmrBehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('amr_bt_node')
        self.get_logger().info("Initializing AMR Behavior Tree Central Node...")

        # 1. Blackboard Initialization
        self.bb = py_trees.blackboard.Client(name="AMR_BT_Node_Client")
        self.bb.register_key(key="general_status", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="nav_status", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="home_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="upper_goal", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="gesture_goal", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="cancel_requested", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="save_map_filename", access=py_trees.common.Access.WRITE)

        self.bb.general_status = "IDLE"
        self.bb.nav_status = "IDLE"
        self.bb.home_pose = read_stored_home()
        self.bb.upper_goal = None
        self.bb.gesture_goal = None
        self.bb.cancel_requested = False
        self.bb.save_map_filename = None

        self._last_published_status = None

        # 2. Publishers
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.status_pub = self.create_publisher(String, '/general_status', status_qos)

        # 3. Subscribers
        # Gesture goals from YOLOv11 Pose Detector
        self.gesture_sub = self.create_subscription(
            PoseStamped,
            '/yolo/gesture_goal',
            self.gesture_goal_callback,
            10
        )

        # Commands from Radxa Tầng Trên or manual scripts
        self.cmd_sub = self.create_subscription(
            String,
            '/amr/command',
            self.command_callback,
            10
        )

        # RViz 2D Goal Pose listener
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_goal_callback,
            10
        )

        # Legacy /general_status listener for 'COMEBACK' command
        self.legacy_cmd_sub = self.create_subscription(
            String,
            '/general_status_cmd',
            self.legacy_cmd_callback,
            10
        )

        # 4. Create and setup Behavior Tree
        self.bt = create_amr_behavior_tree(self)
        self.bt.setup(timeout=15.0)

        # 5. Tree Ticker Timer (10Hz)
        self.timer = self.create_timer(0.1, self.tree_tick)
        self.get_logger().info("AMR Behavior Tree running at 10Hz.")

    def gesture_goal_callback(self, msg: PoseStamped):
        """Receives target pose from YOLO hand-wave detection."""
        cur_status = getattr(self.bb, "general_status", "IDLE")
        if cur_status in ["IDLE", "SERVE"]:
            self.get_logger().info(
                f"Received gesture goal at x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}. Pushing to Behavior Tree..."
            )
            self.bb.gesture_goal = msg
        else:
            self.get_logger().warn(
                f"Ignored gesture goal because robot is currently in state '{cur_status}'."
            )

    def rviz_goal_callback(self, msg: PoseStamped):
        """Receives goal pose from RViz or external planner."""
        self.get_logger().info(
            f"Received external goal pose at x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}."
        )
        self.bb.upper_goal = {
            "type": "GO_POSE",
            "pose": msg
        }

    def command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"Received AMR command: '{cmd}'")
        if cmd in ["COMEBACK", "GO_HOME", "HOME"]:
            # Reload latest stored home
            self.bb.home_pose = read_stored_home()
            self.bb.upper_goal = {"type": "GO_HOME"}
        elif cmd in ["CANCEL", "STOP", "ABORT"]:
            self.bb.cancel_requested = True
        elif cmd.startswith("SAVE_MAP:"):
            filename = cmd.split(":", 1)[1].strip()
            self.bb.save_map_filename = filename
            self.bb.upper_goal = {"type": "SAVE_MAP"}

    def legacy_cmd_callback(self, msg: String):
        self.command_callback(msg)

    def tree_tick(self):
        """Ticks the Behavior Tree at 10Hz and broadcasts status changes."""
        self.bt.tick()

        # Check and publish status update
        cur_status = getattr(self.bb, "general_status", "IDLE")
        if cur_status != self._last_published_status:
            self._last_published_status = cur_status
            msg = String()
            msg.data = cur_status
            self.status_pub.publish(msg)
            self.get_logger().info(f"AMR State Transition -> [{cur_status}]")


def main(args=None):
    rclpy.init(args=args)
    node = AmrBehaviorTreeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
