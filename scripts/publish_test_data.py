#!/usr/bin/env python3
"""
Test publisher script for Radxa AMR:
Publishes dummy Image, LaserScan, and TF so you can verify RViz2 and rqt_image_view on a remote PC without any hardware attached.
"""

import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

class DummySensorPublisher(Node):
    def __init__(self):
        super().__init__('dummy_sensor_publisher')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.count = 0
        self.get_logger().info("Dummy Sensor Publisher started!")
        self.get_logger().info("Publishing to: /image_raw (rqt_image_view) and /scan + /tf (rviz2)")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.count += 1
        
        # 1. Publish TF: base_link -> laser
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.15
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.20
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Dummy LaserScan (circle of radius 1.5m)
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # 360 points around a circle with subtle wave animation
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        wave_offset = 0.2 * math.sin(self.count * 0.1)
        scan.ranges = [float(1.5 + wave_offset + 0.05 * math.sin(i * 0.1)) for i in range(num_readings)]
        self.scan_pub.publish(scan)

        # 3. Publish Dummy Image (animated test card)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Background gradient
        img[:, :] = (30, 30, 45)
        # Draw frame / box
        cv2.rectangle(img, (20, 20), (620, 460), (0, 200, 100), 2)
        # Text overlay
        cv2.putText(img, "Radxa (ROS 2 Lyrical) -> PC (Jazzy)", (40, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(img, f"Status: CONNECTED & STREAMING", (40, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Frame: {self.count}", (40, 170), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"Timestamp: {time.strftime('%H:%M:%S')}", (40, 220), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        
        # Rotating circle animation
        angle = (self.count * 5) % 360
        cx = int(320 + 80 * math.cos(math.radians(angle)))
        cy = int(350 + 40 * math.sin(math.radians(angle)))
        cv2.circle(img, (cx, cy), 20, (0, 140, 255), -1)

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = now
        msg.header.frame_id = 'camera_frame'
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummySensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
