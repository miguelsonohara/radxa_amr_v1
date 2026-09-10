#!/usr/bin/env python3
"""
slam_behavior.py - SLAM Toolbox and Mode Management Behaviors for BehaviorTree
"""

import os
import subprocess
import threading
import time
import py_trees
from slam_toolbox.srv import SerializePoseGraph, SaveMap
from std_msgs.msg import String


class SaveMapBehavior(py_trees.behaviour.Behaviour):
    """
    Calls SLAM Toolbox serialize_map and save_map services.
    Reads `save_map_filename` from blackboard.
    """
    def __init__(self, name: str, node):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="save_map_filename", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="save_map_result", access=py_trees.common.Access.WRITE)

        self.serialize_client = None
        self.save_grid_client = None

    def setup(self, **kwargs):
        self.serialize_client = self.node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.save_grid_client = self.node.create_client(SaveMap, '/slam_toolbox/save_map')
        return True

    def initialise(self):
        self._is_done = False
        self._success = False
        self._message = ""

        filename = getattr(self.blackboard, "save_map_filename", None)
        if not filename:
            self._is_done = True
            self._success = False
            self._message = "No save_map_filename specified on blackboard"
            return

        # Run async service calls in thread to avoid blocking tree tick
        threading.Thread(target=self._execute_save, args=(filename,), daemon=True).start()

    def _execute_save(self, full_base_path):
        self.node.get_logger().info(f"[{self.name}] Serializing and saving map to: {full_base_path}")

        # 1. Serialize posegraph
        ser_ok = False
        if self.serialize_client.wait_for_service(timeout_sec=3.0):
            req = SerializePoseGraph.Request()
            req.filename = full_base_path
            future = self.serialize_client.call_async(req)
            # Wait up to 5 seconds
            t0 = time.time()
            while time.time() - t0 < 5.0 and not future.done():
                time.sleep(0.1)
            if future.done() and future.result() and future.result().result == 0:
                ser_ok = True
                self.node.get_logger().info(f"[{self.name}] Posegraph serialized successfully.")
        else:
            self.node.get_logger().warn(f"[{self.name}] /slam_toolbox/serialize_map service not ready.")

        # 2. Save 2D occupancy grid
        grid_ok = False
        if self.save_grid_client.wait_for_service(timeout_sec=3.0):
            req = SaveMap.Request()
            req.name = String()
            req.name.data = full_base_path
            future = self.save_grid_client.call_async(req)
            t0 = time.time()
            while time.time() - t0 < 5.0 and not future.done():
                time.sleep(0.1)
            if future.done() and future.result() and future.result().result == 0:
                grid_ok = True
                self.node.get_logger().info(f"[{self.name}] Occupancy grid saved successfully.")
        
        # Fallback to map_saver_cli if save_map service not ready
        if not grid_ok:
            try:
                cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", full_base_path]
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=12)
                if res.returncode == 0:
                    grid_ok = True
                    self.node.get_logger().info(f"[{self.name}] Saved grid via map_saver_cli fallback.")
            except Exception as e:
                self.node.get_logger().warn(f"[{self.name}] Fallback map_saver_cli error: {e}")

        self._success = (ser_ok or grid_ok)
        self._message = f"Map save: serialize={ser_ok}, grid={grid_ok}"
        self._is_done = True

    def update(self) -> py_trees.common.Status:
        if not hasattr(self, '_is_done') or not self._is_done:
            return py_trees.common.Status.RUNNING

        self.blackboard.save_map_result = {"success": self._success, "message": self._message}
        return py_trees.common.Status.SUCCESS if self._success else py_trees.common.Status.FAILURE
