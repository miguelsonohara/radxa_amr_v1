#!/usr/bin/env python3
"""
Odom Scale Corrector
Scales raw wheel odometry from ESP32 to compensate for wheel diameter mismatch
(ESP32 firmware compiled with 65mm while physical wheels are 80mm).
Scale factor = 80mm / 65mm = 1.23076923
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomScaleCorrector(Node):
    def __init__(self):
        super().__init__('odom_scale_corrector')
        self.declare_parameter('scale_factor', 80.0 / 65.0)
        self.scale = float(self.get_parameter('scale_factor').value)

        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom_corrected', 10)
        self.get_logger().info(f'Odom Scale Corrector active with scale_factor: {self.scale:.6f} (80mm / 65mm)')

    def odom_callback(self, msg: Odometry):
        # Scale cumulative position in odom frame
        msg.pose.pose.position.x *= self.scale
        msg.pose.pose.position.y *= self.scale

        # Scale linear velocity in base_link frame
        msg.twist.twist.linear.x *= self.scale
        msg.twist.twist.linear.y *= self.scale

        # Scale covariance variances (Var(a*X) = a^2 * Var(X))
        s2 = self.scale * self.scale
        cov = list(msg.pose.covariance)
        cov[0] *= s2
        cov[7] *= s2
        msg.pose.covariance = tuple(cov)

        t_cov = list(msg.twist.covariance)
        t_cov[0] *= s2
        t_cov[7] *= s2
        msg.twist.covariance = tuple(t_cov)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomScaleCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
