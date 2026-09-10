#!/usr/bin/env python3
"""
nav_behavior.py - Nav2 NavigateToPose Action Behavior for BehaviorTree
"""

import time
import py_trees
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class NavigateToPoseBehavior(py_trees.behaviour.Behaviour):
    """
    Ticks asynchronously while Nav2 executes NavigateToPose.
    Reads target pose from blackboard variable `target_pose_key`.
    """
    def __init__(self, name: str, node, target_pose_key: str = "nav_target_pose", action_name: str = "navigate_to_pose"):
        super().__init__(name)
        self.node = node
        self.target_pose_key = target_pose_key
        self.action_name = action_name
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key=self.target_pose_key, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="nav_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="cancel_requested", access=py_trees.common.Access.READ)

        self.action_client = None
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_accepted = None
        self.result_status = None

    def setup(self, **kwargs):
        self.action_client = ActionClient(self.node, NavigateToPose, self.action_name)
        return True

    def initialise(self):
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.goal_accepted = None
        self.result_status = None

        if not self.blackboard.exists(self.target_pose_key):
            self.node.get_logger().error(f"[{self.name}] Target pose key '{self.target_pose_key}' not found on blackboard!")
            return

        target_pose = getattr(self.blackboard, self.target_pose_key, None)
        if target_pose is None or not isinstance(target_pose, PoseStamped):
            self.node.get_logger().error(f"[{self.name}] Invalid target pose: {target_pose}")
            return

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn(f"[{self.name}] Nav2 action server '{self.action_name}' not available!")
            self.blackboard.nav_status = "FAILED"
            self.goal_accepted = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.node.get_logger().info(
            f"[{self.name}] Dispatching Nav2 goal: frame={target_pose.header.frame_id}, "
            f"x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}"
        )
        self.blackboard.nav_status = "PLANNING"
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn(f"[{self.name}] Nav2 goal REJECTED!")
                self.goal_accepted = False
                self.blackboard.nav_status = "FAILED"
                return

            self.node.get_logger().info(f"[{self.name}] Nav2 goal ACCEPTED, executing...")
            self.goal_handle = goal_handle
            self.goal_accepted = True
            self.blackboard.nav_status = "EXECUTING"

            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self._goal_result_callback)
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Error handling goal response: {e}")
            self.goal_accepted = False

    def _goal_result_callback(self, future):
        try:
            result = future.result()
            self.result_status = result.status
            self.node.get_logger().info(f"[{self.name}] Nav2 goal completed with status: {self.result_status}")
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Error getting result: {e}")
            self.result_status = GoalStatus.STATUS_ABORTED

    def update(self) -> py_trees.common.Status:
        # Check if cancel was requested
        if self.blackboard.exists("cancel_requested") and getattr(self.blackboard, "cancel_requested", False):
            self.node.get_logger().info(f"[{self.name}] Cancel requested via blackboard.")
            self._cancel_goal()
            self.blackboard.nav_status = "IDLE"
            return py_trees.common.Status.FAILURE

        if self.send_goal_future is None:
            return py_trees.common.Status.FAILURE

        if self.goal_accepted is False:
            self.blackboard.nav_status = "FAILED"
            return py_trees.common.Status.FAILURE

        if self.goal_accepted is None:
            # Still waiting for server response
            return py_trees.common.Status.RUNNING

        # Goal is accepted, waiting for result
        if self.result_status is None:
            self.blackboard.nav_status = "EXECUTING"
            return py_trees.common.Status.RUNNING

        if self.result_status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Robot successfully reached destination!")
            self.blackboard.nav_status = "ARRIVED"
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn(f"[{self.name}] Navigation ended with status {self.result_status}")
            self.blackboard.nav_status = "FAILED"
            return py_trees.common.Status.FAILURE

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.node.get_logger().info(f"[{self.name}] Canceling active Nav2 goal...")
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.node.get_logger().warn(f"[{self.name}] Exception canceling goal: {e}")
            self.goal_handle = None

    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.INVALID or new_status == py_trees.common.Status.FAILURE:
            self._cancel_goal()
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
