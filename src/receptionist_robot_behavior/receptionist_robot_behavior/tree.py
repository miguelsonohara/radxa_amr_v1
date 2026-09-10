#!/usr/bin/env python3
"""
tree.py - Constructs the Receptionist Robot AMR Behavior Tree
"""

import math
import numpy as np
import py_trees
from geometry_msgs.msg import PoseStamped

from receptionist_robot_behavior.behaviors.nav_behavior import NavigateToPoseBehavior
from receptionist_robot_behavior.behaviors.slam_behavior import SaveMapBehavior


def get_bb_value(client, key, default=None):
    if client.exists(key):
        try:
            val = getattr(client, key)
            return val if val is not None else default
        except (KeyError, AttributeError):
            return default
    return default


# ==============================================================================
# Helper Conditions & Actions
# ==============================================================================

class CheckCancelRequested(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "CheckCancelRequested"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="cancel_requested", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if get_bb_value(self.blackboard, "cancel_requested", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class ClearCancelAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ClearCancelAction"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="cancel_requested", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gesture_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.blackboard.cancel_requested = False
        self.blackboard.upper_goal = None
        self.blackboard.gesture_goal = None
        self.blackboard.general_status = "IDLE"
        return py_trees.common.Status.SUCCESS


class HasUpperGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "HasUpperGoal"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "upper_goal", None)
        if goal is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CheckUpperGoalType(py_trees.behaviour.Behaviour):
    def __init__(self, goal_type: str, name: str = None):
        super().__init__(name or f"IsGoal_{goal_type}")
        self.goal_type = goal_type
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "upper_goal", None)
        if goal and isinstance(goal, dict) and goal.get("type") == self.goal_type:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PrepareHomeNav(py_trees.behaviour.Behaviour):
    def __init__(self, node, name: str = "PrepareHomeNav"):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="home_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="nav_target_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        home_pose = get_bb_value(self.blackboard, "home_pose", [0.0, 0.0, 0.0])
        x, y, yaw = float(home_pose[0]), float(home_pose[1]), float(home_pose[2])

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.blackboard.nav_target_pose = msg
        self.blackboard.general_status = "RETURNING_HOME"
        self.node.get_logger().info(f"[{self.name}] Target set to Home: x={x}, y={y}, yaw={yaw}")
        return py_trees.common.Status.SUCCESS


class PrepareTargetPoseNav(py_trees.behaviour.Behaviour):
    def __init__(self, node, name: str = "PrepareTargetPoseNav"):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="nav_target_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "upper_goal", None)
        if not goal or not isinstance(goal, dict):
            return py_trees.common.Status.FAILURE

        pose = goal.get("pose")
        if isinstance(pose, PoseStamped):
            self.blackboard.nav_target_pose = pose
        elif isinstance(pose, (list, tuple)) and len(pose) >= 3:
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.blackboard.nav_target_pose = msg
        else:
            return py_trees.common.Status.FAILURE

        self.blackboard.general_status = "NAVIGATING"
        return py_trees.common.Status.SUCCESS


class HandleUpperGoalFailure(py_trees.behaviour.Behaviour):
    def __init__(self, node, name: str = "HandleUpperGoalFailure"):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "upper_goal", None)
        self.node.get_logger().warn(
            f"[{self.name}] Upper goal {goal} execution failed or was rejected. Resetting state to IDLE."
        )
        self.blackboard.upper_goal = None
        self.blackboard.general_status = "IDLE"
        return py_trees.common.Status.SUCCESS


class HandleGestureGoalFailure(py_trees.behaviour.Behaviour):
    def __init__(self, node, name: str = "HandleGestureGoalFailure"):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="gesture_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.node.get_logger().warn(
            f"[{self.name}] Gesture navigation failed or was rejected. Clearing gesture goal and resetting to IDLE."
        )
        self.blackboard.gesture_goal = None
        self.blackboard.general_status = "IDLE"
        return py_trees.common.Status.SUCCESS


class FinishUpperGoal(py_trees.behaviour.Behaviour):
    def __init__(self, final_status: str = "IDLE", name: str = None):
        super().__init__(name or f"FinishUpperGoal_{final_status}")
        self.final_status = final_status
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="upper_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.blackboard.upper_goal = None
        self.blackboard.general_status = self.final_status
        return py_trees.common.Status.SUCCESS


class CheckRobotReadyForGesture(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "CheckRobotReadyForGesture"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        status = get_bb_value(self.blackboard, "general_status", "IDLE")
        if status in ["IDLE", "SERVE"]:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CheckGestureGoalReceived(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "CheckGestureGoalReceived"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="gesture_goal", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "gesture_goal", None)
        if goal is not None and isinstance(goal, PoseStamped):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PrepareGestureNav(py_trees.behaviour.Behaviour):
    def __init__(self, node, name: str = "PrepareGestureNav"):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="gesture_goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="nav_target_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        goal = get_bb_value(self.blackboard, "gesture_goal", None)
        if goal is None or not isinstance(goal, PoseStamped):
            return py_trees.common.Status.FAILURE

        self.blackboard.nav_target_pose = goal
        self.blackboard.general_status = "NAVIGATING"
        self.node.get_logger().info(
            f"[{self.name}] Approaching person gesture goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )
        return py_trees.common.Status.SUCCESS


class FinishGestureNav(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "FinishGestureNav"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="gesture_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="general_status", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.blackboard.gesture_goal = None
        self.blackboard.general_status = "SERVE"
        return py_trees.common.Status.SUCCESS


class IdleTelemetryAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "IdleTelemetry"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS


# ==============================================================================
# Tree Assembly
# ==============================================================================

def create_amr_behavior_tree(node) -> py_trees.trees.BehaviourTree:
    """Creates the root AMR Behavior Tree."""
    # Pre-register and initialize blackboard defaults
    init_client = py_trees.blackboard.Client(name="TreeInitClient")
    init_client.register_key(key="general_status", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="nav_status", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="cancel_requested", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="upper_goal", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="gesture_goal", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="home_pose", access=py_trees.common.Access.WRITE)
    init_client.register_key(key="save_map_filename", access=py_trees.common.Access.WRITE)

    init_client.general_status = "IDLE"
    init_client.nav_status = "IDLE"
    init_client.cancel_requested = False
    init_client.upper_goal = None
    init_client.gesture_goal = None

    # Preserve stored home pose from node if available
    stored_home = [0.0, 0.0, 0.0]
    if hasattr(node, 'bb') and hasattr(node.bb, 'home_pose') and node.bb.home_pose:
        stored_home = node.bb.home_pose
    init_client.home_pose = stored_home
    init_client.save_map_filename = None

    root = py_trees.composites.Selector(name="AMR_Root_Policy", memory=False)

    # --------------------------------------------------------------------------
    # 1. Emergency / Cancel Branch
    # --------------------------------------------------------------------------
    cancel_seq = py_trees.composites.Sequence(name="CancelSequence", memory=True)
    cancel_seq.add_children([
        CheckCancelRequested(name="IsCancelRequested"),
        ClearCancelAction(name="ClearCancelAndResetState")
    ])

    # --------------------------------------------------------------------------
    # 2. Upper-Tier Command Branch (Radxa Tầng Trên / REST)
    # --------------------------------------------------------------------------
    upper_seq = py_trees.composites.Sequence(name="UpperCommandSequence", memory=True)
    upper_seq.add_child(HasUpperGoal(name="HasUpperGoal"))

    upper_router = py_trees.composites.Selector(name="RouteUpperGoal", memory=False)

    # Sub 2a: Go Home
    go_home_seq = py_trees.composites.Sequence(name="GoHomeSequence", memory=True)
    go_home_seq.add_children([
        CheckUpperGoalType(goal_type="GO_HOME", name="IsGoHome"),
        PrepareHomeNav(node, name="PrepHomeNav"),
        NavigateToPoseBehavior(name="ExecHomeNav", node=node),
        FinishUpperGoal(final_status="IDLE", name="FinishHomeNav")
    ])

    # Sub 2b: Go To Pose
    go_pose_seq = py_trees.composites.Sequence(name="GoPoseSequence", memory=True)
    go_pose_seq.add_children([
        CheckUpperGoalType(goal_type="GO_POSE", name="IsGoPose"),
        PrepareTargetPoseNav(node, name="PrepPoseNav"),
        NavigateToPoseBehavior(name="ExecPoseNav", node=node),
        FinishUpperGoal(final_status="SERVE", name="FinishPoseNav")
    ])

    # Sub 2c: Save Map
    save_map_seq = py_trees.composites.Sequence(name="SaveMapSequence", memory=True)
    save_map_seq.add_children([
        CheckUpperGoalType(goal_type="SAVE_MAP", name="IsSaveMap"),
        SaveMapBehavior(name="ExecSaveMap", node=node),
        FinishUpperGoal(final_status="IDLE", name="FinishSaveMap")
    ])

    upper_router.add_children([go_home_seq, go_pose_seq, save_map_seq])

    # Failure fallback: catches any rejection/failure and cleanly resets state to IDLE
    upper_handler = py_trees.composites.Selector(name="UpperGoalHandler", memory=False)
    upper_handler.add_children([
        upper_router,
        HandleUpperGoalFailure(node, name="UpperGoalFailureHandler")
    ])

    upper_seq.add_child(upper_handler)

    # --------------------------------------------------------------------------
    # 3. Gesture / Serve Branch (Camera YOLO)
    # --------------------------------------------------------------------------
    gesture_exec_seq = py_trees.composites.Sequence(name="GestureExecSeq", memory=True)
    gesture_exec_seq.add_children([
        PrepareGestureNav(node, name="PrepGestureNav"),
        NavigateToPoseBehavior(name="ExecGestureNav", node=node),
        FinishGestureNav(name="FinishGestureNav")
    ])

    gesture_handler = py_trees.composites.Selector(name="GestureHandler", memory=False)
    gesture_handler.add_children([
        gesture_exec_seq,
        HandleGestureGoalFailure(node, name="GestureFailureHandler")
    ])

    gesture_seq = py_trees.composites.Sequence(name="GestureSequence", memory=True)
    gesture_seq.add_children([
        CheckRobotReadyForGesture(name="IsReadyForGesture"),
        CheckGestureGoalReceived(name="HasGestureGoal"),
        gesture_handler
    ])

    # --------------------------------------------------------------------------
    # 4. Idle / Telemetry
    # --------------------------------------------------------------------------
    idle_action = IdleTelemetryAction(name="IdleTelemetry")

    # Assemble Root
    root.add_children([cancel_seq, upper_seq, gesture_seq, idle_action])

    tree = py_trees.trees.BehaviourTree(root)
    return tree
