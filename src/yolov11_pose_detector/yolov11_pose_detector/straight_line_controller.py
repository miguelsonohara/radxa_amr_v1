#!/usr/bin/env python3
"""
Straight-line controller for receptionist robot.

Pipeline:
  /person_goal  (YOLO + /scan fusion)  -> end goal
  /scan                                 -> obstacle freeze / resume

Behavior:
  1. Drive straight toward the latest PoseStamped goal (rotate-then-go).
  2. If /scan sees an obstacle in the front sector -> freeze (cmd_vel = 0).
  3. When the path is clear again -> resume remaining motion to the same goal.
"""

import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import tf2_ros


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')

        self.declare_parameter('goal_topic', '/person_goal')
        # RViz "2D Goal Pose" — use this to test drive/freeze without camera/YOLO.
        self.declare_parameter('rviz_goal_topic', '/goal_pose')
        self.declare_parameter('cancel_topic', '/person_goal/cancel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/straight_nav/status')
        self.declare_parameter('frozen_topic', '/straight_nav/frozen')

        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.55)
        self.declare_parameter('yaw_align_threshold_rad', 0.20)
        self.declare_parameter('goal_tolerance_m', 0.20)
        self.declare_parameter('goal_yaw_tolerance_rad', 0.30)

        # /scan obstacle gate (front sector)
        self.declare_parameter('obstacle_stop_range_m', 0.70)
        self.declare_parameter('obstacle_clear_range_m', 0.90)
        self.declare_parameter('obstacle_half_angle_deg', 25.0)
        self.declare_parameter('obstacle_min_hits', 3)
        self.declare_parameter('scan_timeout_sec', 0.5)

        self.declare_parameter('use_stamped_cmd_vel', True)

        goal_topic = self.get_parameter('goal_topic').value
        rviz_goal_topic = self.get_parameter('rviz_goal_topic').value
        cancel_topic = self.get_parameter('cancel_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value
        frozen_topic = self.get_parameter('frozen_topic').value

        self.map_frame = self.get_parameter('map_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.yaw_align_threshold = float(self.get_parameter('yaw_align_threshold_rad').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance_m').value)
        self.goal_yaw_tolerance = float(self.get_parameter('goal_yaw_tolerance_rad').value)
        self.stop_range = float(self.get_parameter('obstacle_stop_range_m').value)
        self.clear_range = float(self.get_parameter('obstacle_clear_range_m').value)
        self.obs_half_angle = math.radians(float(self.get_parameter('obstacle_half_angle_deg').value))
        self.obs_min_hits = int(self.get_parameter('obstacle_min_hits').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.use_stamped = bool(self.get_parameter('use_stamped_cmd_vel').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal: Optional[PoseStamped] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = 0.0
        self.frozen = False
        self.status = 'IDLE'
        self._last_status_log = 0.0
        self._last_scan_warn = 0.0

        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, 10)
        self.create_subscription(PoseStamped, rviz_goal_topic, self._goal_cb, 10)
        self.create_subscription(Bool, cancel_topic, self._cancel_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        if self.use_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.frozen_pub = self.create_publisher(Bool, frozen_topic, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._control_loop)

        self._publish_status('IDLE')
        self.get_logger().info(
            'StraightLineController ready: goals on %s or %s (RViz), obstacle gate on %s'
            % (goal_topic, rviz_goal_topic, scan_topic)
        )

    def _goal_cb(self, msg: PoseStamped):
        self.goal = msg
        self.frozen = False
        self._publish_status('EXECUTING')
        self.get_logger().info(
            'New goal (%.2f, %.2f) in frame %s'
            % (msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
        )

    def _cancel_cb(self, msg: Bool):
        if msg.data:
            self.goal = None
            self.frozen = False
            self._stop()
            self._publish_status('IDLE')
            self.get_logger().info('Goal canceled')

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = time.time()

    def _publish_status(self, status: str):
        self.status = status
        out = String()
        out.data = status
        self.status_pub.publish(out)
        fr = Bool()
        fr.data = self.frozen
        self.frozen_pub.publish(fr)

    def _stop(self):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(Twist())

    def _publish_cmd(self, linear_x: float, angular_z: float):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.twist.linear.x = linear_x
            msg.twist.angular.z = angular_z
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_pub.publish(msg)

    def _front_obstacle(self) -> bool:
        """True if /scan has enough hits in the front sector within stop/clear range."""
        scan = self.latest_scan
        stale = (time.time() - self.latest_scan_time) > self.scan_timeout
        if scan is None or len(scan.ranges) == 0 or stale:
            # Missing or stale /scan must block motion: driving blind at
            # linear_speed is worse than stalling until the lidar recovers.
            now = time.time()
            if now - self._last_scan_warn > 2.0:
                self._last_scan_warn = now
                self.get_logger().warn('No usable /scan — blocking motion')
            return True

        threshold = self.clear_range if self.frozen else self.stop_range
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        hits = 0
        for r, a in zip(scan.ranges, angles):
            if not math.isfinite(r):
                continue
            if abs(normalize_angle(float(a))) > self.obs_half_angle:
                continue
            if scan.range_min <= r <= min(scan.range_max, threshold):
                hits += 1
                if hits >= self.obs_min_hits:
                    return True
        return False

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = yaw_from_quat(t.transform.rotation)
            return x, y, yaw
        except Exception as ex:
            now = time.time()
            if now - self._last_status_log > 2.0:
                self._last_status_log = now
                self.get_logger().warn(f'TF {self.map_frame}->{self.base_frame} unavailable: {ex}')
            return None

    def _control_loop(self):
        if self.goal is None:
            self._stop()
            if self.status != 'IDLE':
                self._publish_status('IDLE')
            return

        pose = self._robot_pose()
        if pose is None:
            self._stop()
            return

        robot_x, robot_y, robot_yaw = pose
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        dx = gx - robot_x
        dy = gy - robot_y
        dist = math.hypot(dx, dy)

        # Position reached: optional final yaw align, then ARRIVED
        if dist <= self.goal_tolerance:
            goal_yaw = yaw_from_quat(self.goal.pose.orientation)
            yaw_err = normalize_angle(goal_yaw - robot_yaw)
            if abs(yaw_err) > self.goal_yaw_tolerance:
                self._publish_cmd(0.0, self.angular_speed if yaw_err > 0.0 else -self.angular_speed)
                return
            self._stop()
            self.goal = None
            self.frozen = False
            self._publish_status('ARRIVED')
            self.get_logger().info('Arrived at goal')
            return

        # Obstacle freeze from /scan only
        blocked = self._front_obstacle()
        if blocked:
            if not self.frozen:
                self.frozen = True
                self._publish_status('FROZEN')
                self.get_logger().info('Obstacle on path (/scan) — freezing roadmap')
            self._stop()
            return

        if self.frozen:
            self.frozen = False
            self._publish_status('EXECUTING')
            self.get_logger().info('Path clear — resuming straight drive')

        desired_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(desired_yaw - robot_yaw)

        # Rotate in place until roughly aligned, then go straight
        if abs(yaw_err) > self.yaw_align_threshold:
            self._publish_cmd(0.0, self.angular_speed if yaw_err > 0.0 else -self.angular_speed)
            return

        # Mild heading correction while driving straight
        ang = max(-self.angular_speed, min(self.angular_speed, 1.5 * yaw_err))
        self._publish_cmd(self.linear_speed, ang)
        if self.status != 'EXECUTING':
            self._publish_status('EXECUTING')


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
