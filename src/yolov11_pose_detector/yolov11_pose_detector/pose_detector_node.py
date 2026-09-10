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
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

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
        self.declare_parameter('camera_info_topic', '/camera_info')
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
        self.declare_parameter('camera_mount_x', -0.045)
        self.declare_parameter('camera_mount_y', 0.0)
        self.declare_parameter('camera_mount_z', 1.07)
        self.declare_parameter('camera_laser_yaw_offset', 0.0)
        self.declare_parameter('camera_laser_yaw_offset_deg', 0.0)
        self.declare_parameter('camera_yaw_offset_deg', 0.0)
        
        self.declare_parameter('person_avg_shoulder_width', 0.38)

        self.declare_parameter('lidar_gate_range_margin', 0.5)
        self.declare_parameter('lidar_gate_angle_deg', 20.0)
        self.declare_parameter('lidar_search_half_angle_deg', 20.0)
        self.declare_parameter('goal_cooldown_sec', 6.0)
        
        self.declare_parameter('arm_raise_head_margin', 0.15)
        self.declare_parameter('arm_raise_elbow_margin', 0.20)

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
        
        laser_offset_rad = self.get_parameter('camera_laser_yaw_offset').value
        laser_offset_deg = self.get_parameter('camera_laser_yaw_offset_deg').value
        self.camera_laser_yaw_offset = float(np.radians(laser_offset_deg)) if abs(laser_offset_deg) > 1e-4 else float(laser_offset_rad)
        self.camera_yaw_offset_rad = float(np.radians(self.get_parameter('camera_yaw_offset_deg').value))
        
        self.person_shoulder_width = self.get_parameter('person_avg_shoulder_width').value
        self.lidar_gate_margin = self.get_parameter('lidar_gate_range_margin').value
        
        search_angle_deg = self.get_parameter('lidar_search_half_angle_deg').value
        gate_angle_deg = self.get_parameter('lidar_gate_angle_deg').value
        self.lidar_search_half_angle_rad = float(np.radians(max(search_angle_deg, gate_angle_deg)))
        self.lidar_gate_angle_rad = self.lidar_search_half_angle_rad
        self.goal_cooldown_sec = self.get_parameter('goal_cooldown_sec').value

        self.arm_raise_head_margin = self.get_parameter('arm_raise_head_margin').value
        self.arm_raise_elbow_margin = self.get_parameter('arm_raise_elbow_margin').value

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
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, qos_profile_sensor_data)
        
        # Publisher for gesture goal to Behavior Tree
        self.gesture_goal_pub = self.create_publisher(PoseStamped, '/yolo/gesture_goal', 10)

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.status_sub = self.create_subscription(String, status_topic, self.general_status_callback, status_qos)
        
        self.latest_scan = None
        self.scan_subscription = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.info_subscription = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.navigation_status = "IDLE"
        self.last_goal_sent_time = 0.0

        # Current Robot State (Updated passively by Behavior Tree via /general_status)
        self.general_status = "IDLE"

        # Temporal Filtering & Goal Smoothing Buffers
        self.declare_parameter('gesture_buffer_size', 10)
        self.declare_parameter('gesture_trigger_threshold', 7)
        self.declare_parameter('smooth_buffer_size', 5)

        gesture_buf_size = self.get_parameter('gesture_buffer_size').value
        self.gesture_trigger_thresh = self.get_parameter('gesture_trigger_threshold').value
        smooth_buf_size = self.get_parameter('smooth_buffer_size').value

        self.gesture_buffer = deque(maxlen=gesture_buf_size)
        self.target_pose_history = deque(maxlen=smooth_buf_size)

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


    # ==========================================================================
    # ROS Callbacks
    # ==========================================================================
    def scan_callback(self, msg):
        self.latest_scan = msg

    def camera_info_callback(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            if not hasattr(self, '_camera_info_received'):
                self._camera_info_received = True
                self.get_logger().info(
                    f"Received valid CameraInfo intrinsics from topic: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}"
                )
            self.f_x = msg.k[0]
            self.f_y = msg.k[4]
            self.c_x = msg.k[2]
            self.c_y = msg.k[5]

            # Extract distortion matrix D if available and precompute undistortion maps
            if len(msg.d) >= 4 and any(abs(val) > 1e-6 for val in msg.d):
                camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
                dist_coeffs = np.array(msg.d, dtype=np.float32)
                h, w = msg.height, msg.width
                if h > 0 and w > 0 and (not hasattr(self, '_undistort_map1') or getattr(self, '_undistort_shape', None) != (w, h)):
                    self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                        camera_matrix, dist_coeffs, None, camera_matrix, (w, h), cv2.CV_32FC1
                    )
                    self._undistort_shape = (w, h)
                    if not hasattr(self, '_undistort_logged'):
                        self._undistort_logged = True
                        self.get_logger().info(f"Initialized OpenCV Undistort maps for image shape ({w}x{h}).")
        else:
            if not hasattr(self, '_camera_info_warned'):
                self._camera_info_warned = True
                self.get_logger().warn("CameraInfo topic received uncalibrated intrinsics (all zeros). Retaining default focal length fallback (fx=550.0).")

    def general_status_callback(self, msg):
        """Passively listens to /general_status from Behavior Tree to update HUD."""
        self.general_status = msg.data.strip().upper()

    # ==========================================================================
    # Main Image Pipeline Callback
    # ==========================================================================
    def image_callback(self, msg):
        start_time = time.time()
        
        try:
            if msg.encoding in ['mjpeg', '8UC1', 'jpeg', 'compressed'] or (len(msg.data) > 0 and msg.encoding not in ['bgr8', 'rgb8']):
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cv_img is None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if cv_img is None:
                self.get_logger().error("Decoded frame is None")
                return

            if hasattr(self, '_undistort_map1') and self._undistort_map1 is not None:
                cv_img = cv2.remap(cv_img, self._undistort_map1, self._undistort_map2, cv2.INTER_LINEAR)
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

        # 2. Process Detections and Smart Multi-Person Selection
        if num_persons > 0:
            target_idx, target_det, raw_waving = self._select_target_person(detections)

            # Update temporal ring buffer with active waving status
            self.gesture_buffer.append(raw_waving)
            num_positive_frames = sum(self.gesture_buffer)
            self.gesture_active = num_positive_frames >= self.gesture_trigger_thresh

            # Process tracking for the selected target person
            target_goal_pose = self._process_person_tracking(target_det['kpts'], target_det['kpts_conf'])

            # Render Visualizations for all detected persons
            for idx, det in enumerate(detections):
                is_selected_waving = (idx == target_idx) and raw_waving
                box_color = COLOR_EMERALD if is_selected_waving else COLOR_AMBER
                label_prefix = "Target (Waving)" if is_selected_waving else "Person"
                self._draw_detection_box(annotated_img, det['box'], det['score'], color=box_color, label_prefix=label_prefix)
                self._draw_skeleton(annotated_img, det['kpts'], det['kpts_conf'])
        else:
            self.gesture_buffer.append(False)
            self.gesture_active = False


        if not self.gesture_active:
            self.target_pose_history.clear()

        cur_time = time.time()

        # 3. Auto-reset FAILED status to IDLE after 5-second failure cooldown
        if self.navigation_status == "FAILED" and (cur_time - self.last_goal_failed_time) > 5.0:
            self.navigation_status = "IDLE"

        # 4. Handle Gesture Goal Publishing to Behavior Tree
        if self.gesture_active and target_goal_pose is not None:
            if (cur_time - self.last_goal_sent_time) > self.goal_cooldown_sec:
                if isinstance(target_goal_pose, PoseStamped):
                    self.last_goal_sent_time = cur_time
                    self.get_logger().info(
                        f"Hand-wave gesture detected! Publishing goal pose to /yolo/gesture_goal: "
                        f"x={target_goal_pose.pose.position.x:.2f}, y={target_goal_pose.pose.position.y:.2f}"
                    )
                    self.gesture_goal_pub.publish(target_goal_pose)
                elif target_goal_pose == "ALREADY_THERE":
                    self.get_logger().info("Robot is already at target safety distance.")

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
    def _draw_detection_box(self, img, box, score, color=COLOR_AMBER, label_prefix="Person"):
        """Draws tech-style bracket corners and overlay label with customizable color and prefix."""
        x1, y1, x2, y2 = box
        box_w, box_h = x2 - x1, y2 - y1
        bracket_len = int(min(box_w, box_h) * 0.15)

        # Corners
        cv2.line(img, (x1, y1), (x1 + bracket_len, y1), color, 2)
        cv2.line(img, (x1, y1), (x1, y1 + bracket_len), color, 2)
        cv2.line(img, (x1 + box_w, y1), (x1 + box_w - bracket_len, y1), color, 2)
        cv2.line(img, (x1 + box_w, y1), (x1 + box_w, y1 + bracket_len), color, 2)
        cv2.line(img, (x1, y1 + box_h), (x1 + bracket_len, y1 + box_h), color, 2)
        cv2.line(img, (x1, y1 + box_h), (x1, y1 + box_h - bracket_len), color, 2)
        cv2.line(img, (x1 + box_w, y1 + box_h), (x1 + box_w - bracket_len, y1 + box_h), color, 2)
        cv2.line(img, (x1 + box_w, y1 + box_h), (x1 + box_w, y1 + box_h - bracket_len), color, 2)

        # Semi-transparent overlay
        overlay = img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        cv2.addWeighted(overlay, 0.1, img, 0.9, 0, img)

        # Label tag
        label = f"{label_prefix}: {score:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - lh - 8), (x1 + lw + 12, y1), color, -1)
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
    def _is_arm_raised(self, kpts, kpts_conf):
        """
        Evaluates single-frame arm gesture with strict geometry criteria:
        1. Calculates spatial scale (shoulder width) and head position (head_top_y, head_center_x).
        2. Filters out false positives like holding/hugging head with 2 hands ("ôm đầu").
        3. Filters out single hand resting on head/face.
        4. Requires raised wrist to be significantly above head top, elbow, and shoulder.
        """
        l_sh_conf = kpts_conf[5] if len(kpts_conf) > 5 else 0.0
        r_sh_conf = kpts_conf[6] if len(kpts_conf) > 6 else 0.0
        l_el_conf = kpts_conf[7] if len(kpts_conf) > 7 else 0.0
        r_el_conf = kpts_conf[8] if len(kpts_conf) > 8 else 0.0
        l_wr_conf = kpts_conf[9] if len(kpts_conf) > 9 else 0.0
        r_wr_conf = kpts_conf[10] if len(kpts_conf) > 10 else 0.0

        # Require at least one shoulder to be valid
        if l_sh_conf <= self.kp_conf_threshold and r_sh_conf <= self.kp_conf_threshold:
            return False

        # Calculate spatial scale (shoulder_dist)
        if l_sh_conf > self.kp_conf_threshold and r_sh_conf > self.kp_conf_threshold:
            dx_sh = kpts[5, 0] - kpts[6, 0]
            dy_sh = kpts[5, 1] - kpts[6, 1]
            shoulder_dist = max(float(np.sqrt(dx_sh**2 + dy_sh**2)), 20.0)
            shoulder_avg_y = (kpts[5, 1] + kpts[6, 1]) / 2.0
            head_center_x = (kpts[5, 0] + kpts[6, 0]) / 2.0
        elif l_sh_conf > self.kp_conf_threshold:
            shoulder_dist = 60.0
            shoulder_avg_y = kpts[5, 1]
            head_center_x = kpts[5, 0]
        else:
            shoulder_dist = 60.0
            shoulder_avg_y = kpts[6, 1]
            head_center_x = kpts[6, 0]

        # Calculate Head Center & Head Top (smaller y = higher in image)
        head_kp_indices = [0, 1, 2, 3, 4]  # Nose, L_Eye, R_Eye, L_Ear, R_Ear
        valid_head_y = [kpts[idx, 1] for idx in head_kp_indices if idx < len(kpts_conf) and kpts_conf[idx] > self.kp_conf_threshold and (kpts[idx, 0] > 1e-3 or kpts[idx, 1] > 1e-3)]
        valid_head_x = [kpts[idx, 0] for idx in head_kp_indices if idx < len(kpts_conf) and kpts_conf[idx] > self.kp_conf_threshold and (kpts[idx, 0] > 1e-3 or kpts[idx, 1] > 1e-3)]

        if len(valid_head_x) > 0:
            head_center_x = float(np.mean(valid_head_x))

        if len(valid_head_y) > 0:
            head_top_y = min(valid_head_y) - 0.15 * shoulder_dist
        else:
            head_top_y = shoulder_avg_y - 0.65 * shoulder_dist

        head_margin = getattr(self, 'arm_raise_head_margin', 0.15)
        elbow_margin = getattr(self, 'arm_raise_elbow_margin', 0.20)

        # ----------------------------------------------------------------------
        # Anti-False-Positive 1: Reject 2 Hands Holding/Clutching Head ("Ôm đầu")
        # ----------------------------------------------------------------------
        if l_wr_conf > self.kp_conf_threshold and r_wr_conf > self.kp_conf_threshold:
            l_wr_x, l_wr_y = kpts[9, 0], kpts[9, 1]
            r_wr_x, r_wr_y = kpts[10, 0], kpts[10, 1]
            l_near_head = (abs(l_wr_x - head_center_x) < 0.75 * shoulder_dist) and (l_wr_y < shoulder_avg_y + 0.2 * shoulder_dist)
            r_near_head = (abs(r_wr_x - head_center_x) < 0.75 * shoulder_dist) and (r_wr_y < shoulder_avg_y + 0.2 * shoulder_dist)
            if l_near_head and r_near_head:
                return False

        # ----------------------------------------------------------------------
        # Evaluate Left Arm Raise
        # ----------------------------------------------------------------------
        left_raised = False
        if l_wr_conf > self.kp_conf_threshold and l_sh_conf > self.kp_conf_threshold:
            wrist_x, wrist_y = kpts[9, 0], kpts[9, 1]
            shoulder_y = kpts[5, 1]

            # Must be significantly above head top and shoulder
            if wrist_y < (head_top_y - head_margin * shoulder_dist) and wrist_y < (shoulder_y - 0.60 * shoulder_dist):
                if l_el_conf > self.kp_conf_threshold:
                    elbow_y = kpts[7, 1]
                    if wrist_y < (elbow_y - elbow_margin * shoulder_dist):
                        # Reject if wrist is resting directly over head center
                        if not (abs(wrist_x - head_center_x) < 0.35 * shoulder_dist and wrist_y > (head_top_y - 0.35 * shoulder_dist)):
                            left_raised = True
                else:
                    if not (abs(wrist_x - head_center_x) < 0.35 * shoulder_dist and wrist_y > (head_top_y - 0.35 * shoulder_dist)):
                        left_raised = True

        # ----------------------------------------------------------------------
        # Evaluate Right Arm Raise
        # ----------------------------------------------------------------------
        right_raised = False
        if r_wr_conf > self.kp_conf_threshold and r_sh_conf > self.kp_conf_threshold:
            wrist_x, wrist_y = kpts[10, 0], kpts[10, 1]
            shoulder_y = kpts[6, 1]

            # Must be significantly above head top and shoulder
            if wrist_y < (head_top_y - head_margin * shoulder_dist) and wrist_y < (shoulder_y - 0.60 * shoulder_dist):
                if r_el_conf > self.kp_conf_threshold:
                    elbow_y = kpts[8, 1]
                    if wrist_y < (elbow_y - elbow_margin * shoulder_dist):
                        # Reject if wrist is resting directly over head center
                        if not (abs(wrist_x - head_center_x) < 0.35 * shoulder_dist and wrist_y > (head_top_y - 0.35 * shoulder_dist)):
                            right_raised = True
                else:
                    if not (abs(wrist_x - head_center_x) < 0.35 * shoulder_dist and wrist_y > (head_top_y - 0.35 * shoulder_dist)):
                        right_raised = True

        return left_raised or right_raised

    def _select_target_person(self, detections):
        """
        Smart Multi-Person Selection:
        Scans all detected persons, identifies candidates with raised arms,
        and selects the candidate closest to the camera frame center (c_x).
        Returns: (target_index, target_detection_dict, is_any_person_waving)
        """
        waving_candidates = []
        for idx, det in enumerate(detections):
            if self._is_arm_raised(det['kpts'], det['kpts_conf']):
                box = det['box']
                center_x = (box[0] + box[2]) / 2.0
                dist_to_center = abs(center_x - self.c_x)
                waving_candidates.append((dist_to_center, idx, det))

        if len(waving_candidates) > 0:
            # Sort by proximity to center of FOV
            waving_candidates.sort(key=lambda item: item[0])
            best_idx = waving_candidates[0][1]
            best_det = waving_candidates[0][2]
            return best_idx, best_det, True

        # Fallback to primary detection if no arm raised
        return 0, detections[0], False


    def _compute_body_centroid(self, kpts, kpts_conf):
        """
        Computes a stable horizontal and vertical body centroid (u, v) in image pixels.
        Robustly combines facial landmarks (nose/eyes/ears), shoulder midpoints, and hip midpoints
        so that asymmetric hand/arm raising gestures do not drag the centroid sideways.
        """
        # 1. Head Centerline (Nose 0, L_Eye 1, R_Eye 2, L_Ear 3, R_Ear 4)
        head_indices = [0, 1, 2, 3, 4]
        valid_head_x = [
            float(kpts[i, 0]) for i in head_indices
            if i < len(kpts_conf) and kpts_conf[i] > self.kp_conf_threshold and (kpts[i, 0] > 1e-3 or kpts[i, 1] > 1e-3)
        ]
        head_center_x = float(np.mean(valid_head_x)) if len(valid_head_x) > 0 else None

        # 2. Shoulder Centerline & Vertical Position (L_Shoulder 5, R_Shoulder 6)
        l_sh_conf = kpts_conf[5] if len(kpts_conf) > 5 else 0.0
        r_sh_conf = kpts_conf[6] if len(kpts_conf) > 6 else 0.0
        
        shoulder_center_x = None
        shoulder_center_y = None
        if l_sh_conf > self.kp_conf_threshold and r_sh_conf > self.kp_conf_threshold:
            shoulder_center_x = (float(kpts[5, 0]) + float(kpts[6, 0])) / 2.0
            shoulder_center_y = (float(kpts[5, 1]) + float(kpts[6, 1])) / 2.0
        elif l_sh_conf > self.kp_conf_threshold:
            shoulder_center_x = float(kpts[5, 0])
            shoulder_center_y = float(kpts[5, 1])
        elif r_sh_conf > self.kp_conf_threshold:
            shoulder_center_x = float(kpts[6, 0])
            shoulder_center_y = float(kpts[6, 1])

        # 3. Hip Centerline & Vertical Position (L_Hip 11, R_Hip 12)
        l_hip_conf = kpts_conf[11] if len(kpts_conf) > 11 else 0.0
        r_hip_conf = kpts_conf[12] if len(kpts_conf) > 12 else 0.0

        hip_center_x = None
        hip_center_y = None
        if l_hip_conf > self.kp_conf_threshold and r_hip_conf > self.kp_conf_threshold:
            hip_center_x = (float(kpts[11, 0]) + float(kpts[12, 0])) / 2.0
            hip_center_y = (float(kpts[11, 1]) + float(kpts[12, 1])) / 2.0
        elif l_hip_conf > self.kp_conf_threshold:
            hip_center_x = float(kpts[11, 0])
            hip_center_y = float(kpts[11, 1])
        elif r_hip_conf > self.kp_conf_threshold:
            hip_center_x = float(kpts[12, 0])
            hip_center_y = float(kpts[12, 1])

        # 4. Fuse Horizontal Centerline (u)
        # Prioritize median/weighted combination of Head, Shoulder, and Hip centers
        centerline_candidates = []
        if shoulder_center_x is not None:
            centerline_candidates.append(shoulder_center_x)
        if hip_center_x is not None:
            centerline_candidates.append(hip_center_x)
        if head_center_x is not None:
            centerline_candidates.append(head_center_x)

        if len(centerline_candidates) == 3:
            # Use median to strongly reject any outlier caused by single-side arm movement
            body_center_u = float(np.median(centerline_candidates))
        elif len(centerline_candidates) == 2:
            body_center_u = float(np.mean(centerline_candidates))
        elif len(centerline_candidates) == 1:
            body_center_u = float(centerline_candidates[0])
        else:
            body_center_u = float(self.c_x)

        # 5. Fuse Vertical Centerline (v)
        if shoulder_center_y is not None and hip_center_y is not None:
            body_center_v = (shoulder_center_y + hip_center_y) / 2.0
        elif shoulder_center_y is not None:
            body_center_v = shoulder_center_y
        else:
            body_center_v = float(self.c_y)

        return body_center_u, body_center_v


    def _process_person_tracking(self, kpts, kpts_conf):
        """Estimates distance, fuses LiDAR range data, and projects Nav2 target pose with moving average smoothing."""
        left_shoulder = kpts[5]
        right_shoulder = kpts[6]

        if not (kpts_conf[5] > self.kp_conf_threshold and kpts_conf[6] > self.kp_conf_threshold):
            return None

        dx = left_shoulder[0] - right_shoulder[0]
        dy = left_shoulder[1] - right_shoulder[1]
        d_pixel = np.sqrt(dx**2 + dy**2)

        if d_pixel <= 5.0:
            return None

        # 1. Monocular Coarse Range Estimation
        Z_est = (self.person_shoulder_width * self.f_x) / d_pixel
        Z_est = float(np.clip(Z_est, 0.8, 8.0))

        # Calculate robust Torso Centroid
        body_center_u, body_center_v = self._compute_body_centroid(kpts, kpts_conf)

        # 2. Wide-Sector LiDAR Cluster Tracking
        best_x_laser, best_y_laser, fused_dist = self._fuse_lidar_distance(body_center_u, body_center_v, Z_est)
        self.fused_distance = fused_dist

        if best_x_laser is not None and best_y_laser is not None:
            # 3A. Direct LiDAR to Map Transformation (100% accurate coordinates from LiDAR point cloud)
            target_coords = self._transform_laser_to_map(best_x_laser, best_y_laser)
        else:
            # 3B. Monocular Camera 3D to Map Transformation Fallback
            tan_x = (body_center_u - self.c_x) / self.f_x
            tan_y = (body_center_v - self.c_y) / self.f_y

            if abs(getattr(self, 'camera_yaw_offset_rad', 0.0)) > 1e-4:
                theta_x = np.arctan(tan_x) - self.camera_yaw_offset_rad
                tan_x = float(np.tan(theta_x))

            norm_factor = np.sqrt(1.0 + tan_x**2 + tan_y**2)
            Z_cam = fused_dist / norm_factor
            X_cam = Z_cam * tan_x
            Y_cam = Z_cam * tan_y
            target_coords = self._transform_camera_to_map(X_cam, Y_cam, Z_cam)

        if target_coords is None:
            return None

        # 4. Moving Average Position Smoothing (5-frame filter)
        self.target_pose_history.append(target_coords)
        avg_x = float(np.mean([pt[0] for pt in self.target_pose_history]))
        avg_y = float(np.mean([pt[1] for pt in self.target_pose_history]))

        # 5. Safety Offset Goal Pose Generation
        return self._generate_safety_goal_pose(avg_x, avg_y)


    def _fuse_lidar_distance(self, body_center_u, body_center_v, Z_est):
        """
        Wide-Sector LiDAR Cluster Tracking:
        1. Projects camera sight ray to laser azimuth angle theta_laser.
        2. Scans a wide sector (+/- lidar_search_half_angle_rad, default +/-20 deg).
        3. Extracts 2D point clusters (Euclidean jump distance < 0.35m).
        4. Selects the most plausible human foreground cluster.
        5. Returns (best_x_laser, best_y_laser, best_range).
        If no cluster is found, falls back to (None, None, Z_est).
        """
        self.tracker_mode = "Mono Est"
        if self.latest_scan is None:
            return None, None, Z_est

        # 1. Project sight ray into camera optical frame at estimated distance Z_est
        tan_x = (body_center_u - self.c_x) / self.f_x
        tan_y = (body_center_v - self.c_y) / self.f_y
        
        p_cam = PointStamped()
        p_cam.header.frame_id = self.camera_frame_id
        p_cam.header.stamp = rclpy.time.Time().to_msg()
        p_cam.point.x = Z_est * tan_x
        p_cam.point.y = Z_est * tan_y
        p_cam.point.z = Z_est

        # 2. Transform sight ray point from camera optical frame to laser frame using TF2
        theta_laser = None
        try:
            if self.tf_buffer.can_transform(self.laser_frame_id, self.camera_frame_id, rclpy.time.Time()):
                p_laser = self.tf_buffer.transform(p_cam, self.laser_frame_id)
                # In laser frame (x forward, y left), angle is arctan2(y, x)
                theta_laser = np.arctan2(p_laser.point.y, p_laser.point.x)
        except Exception as tf_ex:
            self.get_logger().warn(f"Sight ray TF transform camera->laser failed: {tf_ex}")

        # Fallback angle estimation if TF fails
        if theta_laser is None:
            theta_cam = np.arctan2(body_center_u - self.c_x, self.f_x)
            theta_laser = -theta_cam

        # Apply optional camera-laser yaw calibration offset
        if abs(getattr(self, 'camera_laser_yaw_offset', 0.0)) > 1e-4:
            theta_laser += self.camera_laser_yaw_offset

        # 3. Extract points in the wide search sector (+/-20 deg)
        scan = self.latest_scan
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        angles_norm = np.arctan2(np.sin(angles), np.cos(angles))

        angle_diffs = np.arctan2(
            np.sin(angles_norm - theta_laser),
            np.cos(angles_norm - theta_laser)
        )

        search_rad = getattr(self, 'lidar_search_half_angle_rad', np.radians(20.0))
        pts_in_sector = []

        for idx in range(len(scan.ranges)):
            if abs(angle_diffs[idx]) <= search_rad:
                r = scan.ranges[idx]
                if scan.range_min <= r <= min(scan.range_max, 7.5):
                    x_pt = r * np.cos(angles_norm[idx])
                    y_pt = r * np.sin(angles_norm[idx])
                    pts_in_sector.append({
                        'r': float(r),
                        'theta': float(angles_norm[idx]),
                        'x': float(x_pt),
                        'y': float(y_pt),
                        'angle_diff': float(abs(angle_diffs[idx]))
                    })

        if len(pts_in_sector) == 0:
            return None, None, Z_est

        # 4. Cluster adjacent points (Euclidean jump < 0.35m)
        clusters = []
        current_cluster = [pts_in_sector[0]]

        for i in range(1, len(pts_in_sector)):
            prev_pt = pts_in_sector[i - 1]
            curr_pt = pts_in_sector[i]
            d_jump = np.sqrt((curr_pt['x'] - prev_pt['x'])**2 + (curr_pt['y'] - prev_pt['y'])**2)
            if d_jump < 0.35:
                current_cluster.append(curr_pt)
            else:
                if len(current_cluster) >= 2:
                    clusters.append(current_cluster)
                current_cluster = [curr_pt]

        if len(current_cluster) >= 2:
            clusters.append(current_cluster)

        # Fallback if no multi-point clusters: treat entire candidate set as 1 cluster if dense
        if len(clusters) == 0:
            clusters.append(pts_in_sector)

        # 5. Evaluate and Score clusters to pick the human
        scored_clusters = []
        for cl in clusters:
            cl_x = [pt['x'] for pt in cl]
            cl_y = [pt['y'] for pt in cl]
            cl_r = [pt['r'] for pt in cl]
            
            centroid_x = float(np.median(cl_x))
            centroid_y = float(np.median(cl_y))
            mean_r = float(np.median(cl_r))
            
            # Cluster diameter/width
            min_x, max_x = min(cl_x), max(cl_x)
            min_y, max_y = min(cl_y), max(cl_y)
            width = float(np.sqrt((max_x - min_x)**2 + (max_y - min_y)**2))
            
            # Reject clusters wider than 0.85m (e.g. continuous long walls)
            if width > 0.85 and len(clusters) > 1:
                continue

            # Centroid angle difference to camera ray
            cl_theta = np.arctan2(centroid_y, centroid_x)
            cl_angle_err = abs(np.arctan2(np.sin(cl_theta - theta_laser), np.cos(cl_theta - theta_laser)))
            
            # Score: combination of foreground proximity (smaller r is better), angle alignment, and Z_est consistency
            score = mean_r * 0.6 + cl_angle_err * 2.5 + abs(mean_r - Z_est) * 0.2
            scored_clusters.append((score, centroid_x, centroid_y, mean_r))

        if len(scored_clusters) > 0:
            scored_clusters.sort(key=lambda item: item[0])
            best_x = scored_clusters[0][1]
            best_y = scored_clusters[0][2]
            best_r = scored_clusters[0][3]
            
            self.tracker_mode = "LiDAR Cluster"
            self.fused_distance = best_r
            return best_x, best_y, best_r

        return None, None, Z_est


    def _transform_laser_to_map(self, x_laser, y_laser):
        """Transforms 2D laser coordinates (x, y) directly to (map_x, map_y)."""
        point_laser = PointStamped()
        point_laser.header.frame_id = self.laser_frame_id
        point_laser.header.stamp = rclpy.time.Time().to_msg()
        point_laser.point.x = float(x_laser)
        point_laser.point.y = float(y_laser)
        point_laser.point.z = 0.0

        # Direct Transform: laser_frame -> map
        try:
            if self.tf_buffer.can_transform(self.map_frame_id, self.laser_frame_id, rclpy.time.Time()):
                point_map = self.tf_buffer.transform(point_laser, self.map_frame_id)
                return (point_map.point.x, point_map.point.y)
        except Exception as ex:
            self.get_logger().warn(f"Direct laser-to-map transform failed: {ex}")

        # Fallback Transform: base_link -> map
        try:
            if self.tf_buffer.can_transform(self.map_frame_id, self.base_frame_id, rclpy.time.Time()):
                point_base = PointStamped()
                point_base.header.frame_id = self.base_frame_id
                point_base.header.stamp = rclpy.time.Time().to_msg()
                point_base.point.x = float(x_laser)
                point_base.point.y = float(y_laser)
                point_base.point.z = 0.16
                point_map = self.tf_buffer.transform(point_base, self.map_frame_id)
                return (point_map.point.x, point_map.point.y)
        except Exception as tf_ex:
            self.get_logger().warn(f"Fallback laser-to-map transform failed: {tf_ex}")

        return None


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

            if dist_to_target <= (self.safety_distance + 0.10):
                return "ALREADY_THERE"

            ratio = (dist_to_target - self.safety_distance) / dist_to_target
            goal_x = robot_x + dx_map * ratio
            goal_y = robot_y + dy_map * ratio

            dist_to_goal = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            if dist_to_goal < 0.20:
                return "ALREADY_THERE"

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
