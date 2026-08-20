import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OpenCVCamNode(Node):
    def __init__(self):
        super().__init__('usb_cam')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30.0)
        self.declare_parameter('camera_frame_id', 'laser')

        device = self.get_parameter('video_device').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        fps = self.get_parameter('framerate').value
        self.frame_id = self.get_parameter('camera_frame_id').value

        # Parse '/dev/video0' -> index 0 for OpenCV V4L2
        if isinstance(device, str) and device.startswith('/dev/video'):
            try:
                device_idx = int(device.replace('/dev/video', ''))
            except ValueError:
                device_idx = device
        else:
            device_idx = device

        self.get_logger().info(f"Opening OpenCV V4L2 Camera device: {device} ({device_idx}) at {self.width}x{self.height} @ {fps} FPS")
        self.cap = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(device_idx)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        timer_period = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("OpenCV Camera Publisher Node started successfully!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret and frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("Failed to capture frame from camera device.", throttle_duration_sec=5.0)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
