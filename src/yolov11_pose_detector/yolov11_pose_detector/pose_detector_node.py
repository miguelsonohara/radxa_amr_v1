#!/usr/bin/env python3
"""
YOLOv11 Pose Detector & Sensor Fusion Node
-----------------------------------------
Refactored for ROS 2 Jazzy:
- Clean modular structure with dedicated helper methods
- All magic numbers and paths exposed as configurable ROS 2 parameters
- Inter-robot state machine via /general_status (IDLE, SERVE, COMEBACK -> RETURNING_HOME -> IDLE)
"""

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from ultralytics import YOLO

try:
    import onnxruntime as ort
    HAS_ONNXRUNTIME = True
except ImportError:
    HAS_ONNXRUNTIME = False


# ==============================================================================
# Global Visual Constants
# ==============================================================================
COLOR_NEON_BLUE = (255, 191, 0)
COLOR_NEON_PINK = (203, 192, 255)
COLOR_EMERALD   = (87, 207, 80)
COLOR_AMBER     = (0, 165, 255)
COLOR_DARK_GRAY = (40, 40, 40)
COLOR_WHITE     = (255, 255, 255)
COLOR_CRIMSON   = (45, 0, 220)

SKELETON_CONNECTIONS = [
    (0, 1), (0, 2), (1, 3), (2, 4),      # Facial landmarks
    (5, 6),                              # Shoulders
    (5, 7), (7, 9),                      # Left arm
    (6, 8), (8, 10),                     # Right arm
    (5, 11), (6, 12), (11, 12),          # Torso / Hips
    (11, 13), (13, 15),                  # Left leg
    (12, 14), (14, 16)                   # Right leg
]


class YoloV11PoseDetectorNode(Node):
    def __init__(self):
        super().__init__('yolov11_pose_detector')
        
        # ----------------------------------------------------------------------
        # 1. Parameter Declarations
        # ----------------------------------------------------------------------
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/yolov11_pose/debug_image')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('nav_action_server', '/navigate_to_pose')
        self.declare_parameter('status_topic', '/general_status')
        
        self.declare_parameter('workspace_dir', '/home/radxa/receptionist_robot_ws')
        self.declare_parameter('model_name', 'yolo11n-pose.onnx')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('kp_conf_threshold', 0.4)
        self.declare_parameter('onnx_input_size', 320)
        self.declare_parameter('onnx_threads', 4)
        
        self.declare_parameter('focal_length_x', 550.0)
        self.declare_parameter('focal_length_y', 550.0)
        self.declare_parameter('center_x', 320.0)
        self.declare_parameter('center_y', 240.0)
        
        self.declare_parameter('camera_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('laser_frame_id', 'laser')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('camera_mount_x', 0.18)
        self.declare_parameter('camera_mount_y', 0.0)
        self.declare_parameter('camera_mount_z', 0.50)
        self.declare_parameter('camera_laser_yaw_offset', 0.0)
        
        self.declare_parameter('person_avg_shoulder_width', 0.4)
        self.declare_parameter('lidar_gate_range_margin', 0.5)
        self.declare_parameter('lidar_gate_angle_deg', 5.0)
        self.declare_parameter('goal_cooldown_sec', 6.0)
        
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)

        # ----------------------------------------------------------------------
        # 2. Retrieve Parameter Values
        # ----------------------------------------------------------------------
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        nav_action_server = self.get_parameter('nav_action_server').value
        status_topic = self.get_parameter('status_topic').value
        
        self.workspace_dir = self.get_parameter('workspace_dir').value
        self.model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.kp_conf_threshold = self.get_parameter('kp_conf_threshold').value
        self.onnx_input_size = self.get_parameter('onnx_input_size').value
        self.onnx_threads = self.get_parameter('onnx_threads').value
        
        self.f_x = self.get_parameter('focal_length_x').value
        self.f_y = self.get_parameter('focal_length_y').value
        self.c_x = self.get_parameter('center_x').value
        self.c_y = self.get_parameter('center_y').value
        
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.laser_frame_id = self.get_parameter('laser_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.camera_mount_x = self.get_parameter('camera_mount_x').value
        self.camera_mount_y = self.get_parameter('camera_mount_y').value
        self.camera_mount_z = self.get_parameter('camera_mount_z').value
        self.camera_laser_yaw_offset = self.get_parameter('camera_laser_yaw_offset').value
        
        self.person_shoulder_width = self.get_parameter('person_avg_shoulder_width').value
        self.lidar_gate_margin = self.get_parameter('lidar_gate_range_margin').value
        self.lidar_gate_angle_rad = np.radians(self.get_parameter('lidar_gate_angle_deg').value)
        self.goal_cooldown_sec = self.get_parameter('goal_cooldown_sec').value

        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_yaw = self.get_parameter('home_yaw').value

        self.get_logger().info("Initializing YOLOv11 Pose & Sensor Fusion Node...")

        # ----------------------------------------------------------------------
        # 3. Model Initialization
        # ----------------------------------------------------------------------
        self.use_onnx = False
        self.session = None
        self.model = None
        self._init_yolo_model()

        # ----------------------------------------------------------------------
        # 4. ROS Subscriptions, Publishers & Action Client
        # ----------------------------------------------------------------------
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        
        status_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.status_pub = self.create_publisher(String, status_topic, status_qos)
        self.status_sub = self.create_subscription(String, status_topic, self.general_status_callback, status_qos)
        
        self.latest_scan = None
        self.scan_subscription = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.info_subscription = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.nav_client = ActionClient(self, NavigateToPose, nav_action_server)
        self.nav_goal_handle = None
        self.navigation_status = "IDLE"
        self.last_goal_sent_time = 0.0

        # Inter-Robot State Machine Initialization
        self.general_status = "IDLE"
        self.target_type = "NONE"  # "PERSON" or "HOME"
        self._publish_general_status("IDLE")

        # Visual Telemetry Stats
        self.prev_time = 0.0
        self.fps = 0.0
        self.tracker_mode = "Mono Est"
        self.fused_distance = 0.0
        self.gesture_active = False

    # ==========================================================================
    # Initialization & Status Helpers
    # ==========================================================================
    def _init_yolo_model(self):
        """Loads ONNX Runtime session or falls back to PyTorch YOLO."""
        onnx_model_name = self.model_name if self.model_name.endswith('.onnx') else self.model_name.replace('.pt', '.onnx')
        onnx_model_path = os.path.join(self.workspace_dir, onnx_model_name)
        
        if HAS_ONNXRUNTIME:
            if not os.path.exists(onnx_model_path):
                self.get_logger().info("ONNX model not found. Generating from PyTorch...")
                pt_model_name = self.model_name if self.model_name.endswith('.pt') else self.model_name.replace('.onnx', '.pt')
                pt_model_path = os.path.join(self.workspace_dir, pt_model_name)
                if not os.path.exists(pt_model_path):
                    _ = YOLO(pt_model_path)
                model_pt = YOLO(pt_model_path)
                model_pt.export(format='onnx', imgsz=self.onnx_input_size)
                
            if os.path.exists(onnx_model_path):
                self.get_logger().info(f"Loading accelerated ONNX model: {onnx_model_path}")
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = self.onnx_threads
                opts.inter_op_num_threads = self.onnx_threads
                opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                
                self.session = ort.InferenceSession(onnx_model_path, sess_options=opts, providers=['CPUExecutionProvider'])
                self.input_name = self.session.get_inputs()[0].name
                self.use_onnx = True
                self.get_logger().info("ONNX Runtime session initialized successfully!")

        if not self.use_onnx:
            pt_model_name = self.model_name if self.model_name.endswith('.pt') else self.model_name.replace('.onnx', '.pt')
            pt_model_path = os.path.join(self.workspace_dir, pt_model_name)
            self.model = YOLO(pt_model_path)
            self.get_logger().info("Fallback PyTorch model loaded successfully!")

    def _publish_general_status(self, status_str):
        """Updates internal status and publishes string to /general_status."""
        self.general_status = status_str
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"Updated and published /general_status -> '{status_str}'")

    # ==========================================================================
    # ROS Callbacks
    # ==========================================================================
    def scan_callback(self, msg):
        self.latest_scan = msg

    def camera_info_callback(self, msg):
        self.f_x = msg.k[0]
        self.f_y = msg.k[4]
        self.c_x = msg.k[2]
        self.c_y = msg.k[5]

    def general_status_callback(self, msg):
        """Listens to /general_status for inter-robot commands (e.g. 'COMEBACK')."""
        cmd = msg.data.strip().upper()
        if cmd == "COMEBACK":
            if self.general_status != "RETURNING_HOME":
                self.get_logger().info("Received 'COMEBACK' command! Returning to home origin pose (0,0,0)...")
                self._send_home_navigation_goal()

    # ==========================================================================
    # Navigation Action Dispatch & Handlers
    # ==========================================================================
    def send_navigation_goal(self, pose_goal):
        if self.nav_goal_handle is not None:
            self.get_logger().info("Canceling previous navigation target...")
            self.nav_goal_handle.cancel_goal_async()
            
        self.get_logger().info("Dispatching goal target to Nav2...")
        self.navigation_status = "PLANNING"
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server is offline!")
            self.navigation_status = "FAILED"
            if self.target_type == "HOME":
                self._publish_general_status("IDLE")
                self.target_type = "NONE"
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_goal
        
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def _send_home_navigation_goal(self):
        """Constructs home origin pose (0,0,0) and dispatches to Nav2."""
        self.target_type = "HOME"
        self._publish_general_status("RETURNING_HOME")

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.map_frame_id
        home_pose.header.stamp = self.get_clock().now().to_msg()
        home_pose.pose.position.x = self.home_x
        home_pose.pose.position.y = self.home_y
        
        q_z = np.sin(self.home_yaw / 2.0)
        q_w = np.cos(self.home_yaw / 2.0)
        home_pose.pose.orientation.z = q_z
        home_pose.pose.orientation.w = q_w

        self.send_navigation_goal(home_pose)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation target REJECTED by Nav2.")
            self.navigation_status = "FAILED"
            if self.target_type == "HOME":
                self._publish_general_status("IDLE")
                self.target_type = "NONE"
            return
            
        self.get_logger().info("Navigation target ACCEPTED by Nav2.")
        self.nav_goal_handle = goal_handle
        self.navigation_status = "EXECUTING"
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        pass

    def nav_result_callback(self, future):
        status = future.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Robot successfully reached target position!")
            self.navigation_status = "ARRIVED"
            if self.target_type == "PERSON":
                self._publish_general_status("SERVE")
            elif self.target_type == "HOME":
                self.get_logger().info("Arrived at home pose! Transitioning back to IDLE state.")
                self._publish_general_status("IDLE")
                self.target_type = "NONE"
        else:
            self.get_logger().warn(f"Navigation failed (status={status})")
            self.navigation_status = "FAILED"
            if self.target_type == "HOME":
                self._publish_general_status("IDLE")
                self.target_type = "NONE"
        self.nav_goal_handle = None

    # ==========================================================================
    # Main Image Pipeline Callback
    # ==========================================================================
    def image_callback(self, msg):
        start_time = time.time()
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        annotated_img = cv_img.copy()
        orig_h, orig_w = cv_img.shape[:2]
        
        self.gesture_active = False
        target_goal_pose = None

        # 1. Perform Inference
        if self.use_onnx:
            detections = self._run_onnx_inference(cv_img, orig_w, orig_h)
        else:
            detections = self._run_pytorch_inference(cv_img)

        num_persons = len(detections)

        # 2. Process Detections and Render Visualizations
        if num_persons > 0:
            for count, det in enumerate(detections):
                self._draw_detection_box(annotated_img, det['box'], det['score'])
                self._draw_skeleton(annotated_img, det['kpts'], det['kpts_conf'])

                # Process gesture and tracking for the primary (closest/highest score) person
                if count == 0:
                    self.gesture_active = self._check_wrist_gesture(det['kpts'], det['kpts_conf'])
                    target_goal_pose = self._process_person_tracking(det['kpts'], det['kpts_conf'])

        # 3. Handle Navigation Action Triggering (Only if currently IDLE or SERVE, not RETURNING_HOME)
        if self.gesture_active and target_goal_pose is not None and self.general_status != "RETURNING_HOME":
            cur_time = time.time()
            if (cur_time - self.last_goal_sent_time) > self.goal_cooldown_sec:
                self.last_goal_sent_time = cur_time
                self.target_type = "PERSON"
                self.send_navigation_goal(target_goal_pose)

        # 4. Performance FPS & HUD Render
        inf_time_ms = (time.time() - start_time) * 1000.0
        self._update_fps(start_time)
        self._draw_hud(annotated_img, inf_time_ms, num_persons)

        # 5. Publish Output Image
        try:
            pub_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            pub_msg.header = msg.header
            self.publisher.publish(pub_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing failed: {e}")

    # ==========================================================================
    # Inference Helpers
    # ==========================================================================
    def _run_onnx_inference(self, cv_img, orig_w, orig_h):
        """Runs ONNX model inference and converts predictions to a list of detections."""
        img_resized = cv2.resize(cv_img, (self.onnx_input_size, self.onnx_input_size))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_norm = img_rgb.transpose(2, 0, 1)
        img_norm = np.expand_dims(img_norm, axis=0).astype(np.float32) / 255.0

        outputs = self.session.run(None, {self.input_name: img_norm})
        predictions = outputs[0][0].T

        mask = predictions[:, 4] > self.conf_threshold
        predictions = predictions[mask]

        detections = []
        if len(predictions) == 0:
            return detections

        boxes = predictions[:, :4]
        scores = predictions[:, 4]
        kpts = predictions[:, 5:]

        nms_boxes = []
        for box in boxes:
            cx, cy, w, h = box
            x = cx - w / 2.0
            y = cy - h / 2.0
            nms_boxes.append([float(x), float(y), float(w), float(h)])

        indices = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), self.conf_threshold, self.nms_threshold)
        if len(indices) == 0:
            return detections

        if isinstance(indices, np.ndarray):
            indices = indices.flatten()

        x_scale = orig_w / float(self.onnx_input_size)
        y_scale = orig_h / float(self.onnx_input_size)

        for idx in indices:
            box = nms_boxes[idx]
            x1 = int(box[0] * x_scale)
            y1 = int(box[1] * y_scale)
            w_box = int(box[2] * x_scale)
            h_box = int(box[3] * y_scale)

            kp = kpts[idx].reshape(17, 3)
            kpts_scaled = kp[:, :2] * np.array([x_scale, y_scale])
            kpts_conf = kp[:, 2]

            detections.append({
                'box': (x1, y1, x1 + w_box, y1 + h_box),
                'score': float(scores[idx]),
                'kpts': kpts_scaled,
                'kpts_conf': kpts_conf
            })

        return detections

    def _run_pytorch_inference(self, cv_img):
        """Runs PyTorch YOLO fallback inference."""
        results = self.model.predict(source=cv_img, conf=self.conf_threshold, verbose=False)
        detections = []
        if len(results) == 0:
            return detections

        result = results[0]
        boxes = result.boxes
        keypoints = result.keypoints

        if keypoints is None or boxes is None:
            return detections

        for i in range(len(boxes)):
            box = boxes[i].xyxy[0].cpu().numpy().astype(int)
            conf = float(boxes[i].conf[0].cpu().numpy())
            kpts = keypoints.xy[i].cpu().numpy()
            kpts_conf = keypoints.conf[i].cpu().numpy() if keypoints.conf is not None else np.ones(17)

            detections.append({
                'box': (box[0], box[1], box[2], box[3]),
                'score': conf,
                'kpts': kpts,
                'kpts_conf': kpts_conf
            })

        return detections

    # ==========================================================================
    # Visualization Helpers
    # ==========================================================================
    def _draw_detection_box(self, img, box, score):
        """Draws tech-style bracket corners and overlay label."""
        x1, y1, x2, y2 = box
        box_w, box_h = x2 - x1, y2 - y1
        bracket_len = int(min(box_w, box_h) * 0.15)

        # Corners
        cv2.line(img, (x1, y1), (x1 + bracket_len, y1), COLOR_AMBER, 2)
        cv2.line(img, (x1, y1), (x1, y1 + bracket_len), COLOR_AMBER, 2)
        cv2.line(img, (x1 + box_w, y1), (x1 + box_w - bracket_len, y1), COLOR_AMBER, 2)
        cv2.line(img, (x1 + box_w, y1), (x1 + box_w, y1 + bracket_len), COLOR_AMBER, 2)
        cv2.line(img, (x1, y1 + box_h), (x1 + bracket_len, y1 + box_h), COLOR_AMBER, 2)
        cv2.line(img, (x1, y1 + box_h), (x1, y1 + box_h - bracket_len), COLOR_AMBER, 2)
        cv2.line(img, (x1 + box_w, y1 + box_h), (x1 + box_w - bracket_len, y1 + box_h), COLOR_AMBER, 2)
        cv2.line(img, (x1 + box_w, y1 + box_h), (x1 + box_w, y1 + box_h - bracket_len), COLOR_AMBER, 2)

        # Semi-transparent overlay
        overlay = img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), COLOR_AMBER, -1)
        cv2.addWeighted(overlay, 0.1, img, 0.9, 0, img)

        # Label tag
        label = f"Person: {score:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - lh - 8), (x1 + lw + 12, y1), COLOR_AMBER, -1)
        cv2.putText(img, label, (x1 + 6, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1, cv2.LINE_AA)

    def _draw_skeleton(self, img, kpts, kpts_conf):
        """Draws body joints and skeleton connections."""
        for pt1_idx, pt2_idx in SKELETON_CONNECTIONS:
            if pt1_idx < len(kpts) and pt2_idx < len(kpts):
                if kpts_conf[pt1_idx] > self.kp_conf_threshold and kpts_conf[pt2_idx] > self.kp_conf_threshold:
                    x1_k, y1_k = kpts[pt1_idx].astype(int)
                    x2_k, y2_k = kpts[pt2_idx].astype(int)
                    cv2.line(img, (x1_k, y1_k), (x2_k, y2_k), COLOR_NEON_BLUE, 2, cv2.LINE_AA)

        for pt_idx in range(len(kpts)):
            if kpts_conf[pt_idx] > self.kp_conf_threshold:
                xk, yk = kpts[pt_idx].astype(int)
                cv2.circle(img, (xk, yk), 4, COLOR_NEON_PINK, -1, cv2.LINE_AA)
                cv2.circle(img, (xk, yk), 6, COLOR_WHITE, 1, cv2.LINE_AA)

    # ==========================================================================
    # Gesture & Tracking Logic Helpers
    # ==========================================================================
    def _check_wrist_gesture(self, kpts, kpts_conf):
        """Returns True if left or right wrist is raised above nose level."""
        nose_y = kpts[0, 1]
        left_wrist_y = kpts[9, 1]
        right_wrist_y = kpts[10, 1]

        if kpts_conf[0] > self.kp_conf_threshold:
            if (kpts_conf[9] > self.kp_conf_threshold and left_wrist_y < nose_y) or \
               (kpts_conf[10] > self.kp_conf_threshold and right_wrist_y < nose_y):
                return True
        return False

    def _process_person_tracking(self, kpts, kpts_conf):
        """Estimates distance, fuses LiDAR range data, and projects Nav2 target pose."""
        left_shoulder = kpts[5]
        right_shoulder = kpts[6]

        if not (kpts_conf[5] > self.kp_conf_threshold and kpts_conf[6] > self.kp_conf_threshold):
            return None

        dx = left_shoulder[0] - right_shoulder[0]
        dy = left_shoulder[1] - right_shoulder[1]
        d_pixel = np.sqrt(dx**2 + dy**2)

        if d_pixel <= 5.0:
            return None

        # 1. Monocular Distance Estimation from Shoulders
        Z_est = (self.person_shoulder_width * self.f_x) / d_pixel
        shoulder_center_u = (left_shoulder[0] + right_shoulder[0]) / 2.0
        shoulder_center_v = (left_shoulder[1] + right_shoulder[1]) / 2.0
        theta_est = np.arctan2(shoulder_center_u - self.c_x, self.f_x)

        # 2. LiDAR Gating & Sensor Fusion
        d_lidar = self._fuse_lidar_distance(Z_est, theta_est)
        self.fused_distance = d_lidar

        # 3. 3D Coordinates in Camera Optical Frame
        Z_cam = d_lidar
        X_cam = Z_cam * (shoulder_center_u - self.c_x) / self.f_x
        Y_cam = Z_cam * (shoulder_center_v - self.c_y) / self.f_y

        # 4. Map Frame Transformation
        target_coords = self._transform_camera_to_map(X_cam, Y_cam, Z_cam)
        if target_coords is None:
            return None

        # 5. Safety Offset Pose Generation
        return self._generate_safety_goal_pose(target_coords[0], target_coords[1])

    def _fuse_lidar_distance(self, Z_est, theta_est):
        """Gates and returns fused LiDAR distance if valid scan rays are present."""
        self.tracker_mode = "Mono Est"
        if self.latest_scan is None:
            return Z_est

        scan = self.latest_scan
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        angles_norm = np.arctan2(np.sin(angles), np.cos(angles))

        angle_diff = np.arctan2(
            np.sin(angles_norm - (theta_est + self.camera_laser_yaw_offset)),
            np.cos(angles_norm - (theta_est + self.camera_laser_yaw_offset))
        )

        mask_angle = np.abs(angle_diff) <= self.lidar_gate_angle_rad
        valid_ranges = []
        for s_idx in np.where(mask_angle)[0]:
            r = scan.ranges[s_idx]
            if scan.range_min <= r <= scan.range_max:
                if np.abs(r - Z_est) <= self.lidar_gate_margin:
                    valid_ranges.append(r)

        if len(valid_ranges) > 0:
            self.tracker_mode = "LiDAR Fused"
            return np.median(valid_ranges)

        return Z_est

    def _transform_camera_to_map(self, X_cam, Y_cam, Z_cam):
        """Transforms 3D optical camera coordinates to (map_x, map_y)."""
        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame_id
        point_cam.header.stamp = rclpy.time.Time().to_msg()
        point_cam.point.x = X_cam
        point_cam.point.y = Y_cam
        point_cam.point.z = Z_cam

        # Direct Transform: camera_frame -> map
        try:
            if self.tf_buffer.can_transform(self.map_frame_id, self.camera_frame_id, rclpy.time.Time()):
                point_map = self.tf_buffer.transform(point_cam, self.map_frame_id)
                return (point_map.point.x, point_map.point.y)
        except Exception as ex:
            self.get_logger().warn(f"Direct camera-to-map transform failed: {ex}")

        # Fallback Transform: base_link -> map
        X_base = self.camera_mount_x + Z_cam
        Y_base = self.camera_mount_y - X_cam
        point_base = PointStamped()
        point_base.header.frame_id = self.base_frame_id
        point_base.header.stamp = rclpy.time.Time().to_msg()
        point_base.point.x = X_base
        point_base.point.y = Y_base
        point_base.point.z = self.camera_mount_z - Y_cam

        try:
            if self.tf_buffer.can_transform(self.map_frame_id, self.base_frame_id, rclpy.time.Time()):
                point_map = self.tf_buffer.transform(point_base, self.map_frame_id)
                return (point_map.point.x, point_map.point.y)
        except Exception as tf_ex:
            self.get_logger().warn(f"Fallback base-to-map transform failed: {tf_ex}")

        return None

    def _generate_safety_goal_pose(self, target_x_map, target_y_map):
        """Computes safety distance offset position and target yaw orientation."""
        try:
            t_base = self.tf_buffer.lookup_transform(self.map_frame_id, self.base_frame_id, rclpy.time.Time())
            robot_x = t_base.transform.translation.x
            robot_y = t_base.transform.translation.y

            dx_map = target_x_map - robot_x
            dy_map = target_y_map - robot_y
            dist_to_target = np.sqrt(dx_map**2 + dy_map**2)

            if dist_to_target > self.safety_distance:
                ratio = (dist_to_target - self.safety_distance) / dist_to_target
                goal_x = robot_x + dx_map * ratio
                goal_y = robot_y + dy_map * ratio
            else:
                goal_x = robot_x
                goal_y = robot_y

            goal_yaw = np.arctan2(dy_map, dx_map)
            q_z = np.sin(goal_yaw / 2.0)
            q_w = np.cos(goal_yaw / 2.0)

            target_goal_pose = PoseStamped()
            target_goal_pose.header.frame_id = self.map_frame_id
            target_goal_pose.header.stamp = self.get_clock().now().to_msg()
            target_goal_pose.pose.position.x = goal_x
            target_goal_pose.pose.position.y = goal_y
            target_goal_pose.pose.orientation.z = q_z
            target_goal_pose.pose.orientation.w = q_w
            return target_goal_pose
        except Exception as tf_rob_ex:
            self.get_logger().warn(f"Could not retrieve robot pose for goal calculation: {tf_rob_ex}")
            return None

    # ==========================================================================
    # HUD & Performance Tracking
    # ==========================================================================
    def _update_fps(self, start_time):
        """Updates exponential moving average FPS counter."""
        current_time = time.time()
        if self.prev_time > 0.0:
            current_fps = 1.0 / (current_time - self.prev_time)
            self.fps = 0.9 * self.fps + 0.1 * current_fps
        else:
            self.fps = 1.0 / (current_time - start_time)
        self.prev_time = current_time

    def _draw_hud(self, img, inf_time_ms, num_persons):
        """Renders HUD telemetry overlay panel."""
        hud_w, hud_h = 240, 180
        hud_overlay = img.copy()
        cv2.rectangle(hud_overlay, (10, 10), (10 + hud_w, 10 + hud_h), COLOR_DARK_GRAY, -1)
        cv2.addWeighted(hud_overlay, 0.65, img, 0.35, 0, img)

        cv2.rectangle(img, (10, 10), (10 + hud_w, 10 + hud_h), COLOR_EMERALD, 1, cv2.LINE_AA)
        cv2.putText(img, "YOLOv11 SENSOR FUSION", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_EMERALD, 1, cv2.LINE_AA)
        cv2.line(img, (20, 38), (10 + hud_w - 10, 38), COLOR_EMERALD, 1)

        backend_name = "ONNX Runtime (CPU-4T)" if self.use_onnx else "PyTorch (CPU)"
        cv2.putText(img, f"Engine: {backend_name}", (20, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, f"FPS: {self.fps:.1f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, f"Latency: {inf_time_ms:.1f} ms", (20, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)

        dist_str = f"Range: {self.fused_distance:.2f} m ({self.tracker_mode})" if num_persons > 0 else "Range: N/A"
        cv2.putText(img, dist_str, (20, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)

        gest_color = COLOR_EMERALD if self.gesture_active else COLOR_WHITE
        gest_str = "Gesture: TRIGGERED" if self.gesture_active else "Gesture: WAITING"
        cv2.putText(img, gest_str, (20, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.4, gest_color, 1, cv2.LINE_AA)

        status_color = COLOR_EMERALD if self.general_status in ["SERVE", "IDLE"] else COLOR_AMBER
        cv2.putText(img, f"Robot State: {self.general_status}", (20, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1, cv2.LINE_AA)

        nav_color = COLOR_EMERALD if self.navigation_status in ["ARRIVED", "EXECUTING"] else (
            COLOR_CRIMSON if self.navigation_status == "FAILED" else COLOR_WHITE
        )
        cv2.putText(img, f"Nav Status: {self.navigation_status}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, nav_color, 1, cv2.LINE_AA)
        cv2.putText(img, f"Detections: {num_persons}", (20, 166), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = YoloV11PoseDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLOv11 Pose Detector Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
