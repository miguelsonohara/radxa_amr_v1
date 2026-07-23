#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import os

try:
    import onnxruntime as ort
    HAS_ONNXRUNTIME = True
except ImportError:
    HAS_ONNXRUNTIME = False

class YoloV11PoseDetectorNode(Node):
    def __init__(self):
        super().__init__('yolov11_pose_detector')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/yolov11_pose/debug_image')
        self.declare_parameter('model_name', 'yolo11n-pose.onnx')
        self.declare_parameter('conf_threshold', 0.4)
        
        # Camera Calibration / Mounting Parameters
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
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        
        self.f_x = self.get_parameter('focal_length_x').get_parameter_value().double_value
        self.f_y = self.get_parameter('focal_length_y').get_parameter_value().double_value
        self.c_x = self.get_parameter('center_x').get_parameter_value().double_value
        self.c_y = self.get_parameter('center_y').get_parameter_value().double_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.laser_frame_id = self.get_parameter('laser_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        self.camera_mount_x = self.get_parameter('camera_mount_x').get_parameter_value().double_value
        self.camera_mount_y = self.get_parameter('camera_mount_y').get_parameter_value().double_value
        self.camera_mount_z = self.get_parameter('camera_mount_z').get_parameter_value().double_value
        self.camera_laser_yaw_offset = self.get_parameter('camera_laser_yaw_offset').get_parameter_value().double_value
        
        self.get_logger().info(f"Initializing YOLOv11 Pose & Sensor Fusion Node...")
        
        # Initialize YOLO / ONNX model
        workspace_dir = '/home/radxa/receptionist_robot_ws'
        self.use_onnx = False
        
        # If ONNX Runtime is available, automatically load/promote to accelerated ONNX model
        onnx_model_name = model_name if model_name.endswith('.onnx') else model_name.replace('.pt', '.onnx')
        onnx_model_path = os.path.join(workspace_dir, onnx_model_name)
        
        if HAS_ONNXRUNTIME:
            if not os.path.exists(onnx_model_path):
                self.get_logger().info(f"ONNX model not found. Trying to generate from PyTorch...")
                pt_model_name = model_name if model_name.endswith('.pt') else model_name.replace('.onnx', '.pt')
                pt_model_path = os.path.join(workspace_dir, pt_model_name)
                if not os.path.exists(pt_model_path):
                    _ = YOLO(pt_model_path)
                model_pt = YOLO(pt_model_path)
                model_pt.export(format='onnx', imgsz=320)
                
            if os.path.exists(onnx_model_path):
                self.get_logger().info(f"Loading accelerated ONNX model from: {onnx_model_path}")
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = 4
                opts.inter_op_num_threads = 4
                opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                self.session = ort.InferenceSession(onnx_model_path, sess_options=opts, providers=['CPUExecutionProvider'])
                self.input_name = self.session.get_inputs()[0].name
                self.use_onnx = True
                self.get_logger().info("ONNX Runtime session initialized successfully!")
                
        if not self.use_onnx:
            pt_model_name = model_name if model_name.endswith('.pt') else model_name.replace('.onnx', '.pt')
            pt_model_path = os.path.join(workspace_dir, pt_model_name)
            self.model = YOLO(pt_model_path)
            self.get_logger().info("Fallback PyTorch model loaded successfully!")
            
        # CvBridge & Publishers/Subscribers
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        
        # Laser Scan Subscription
        self.latest_scan = None
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Camera Info Subscription (Auto Calibration)
        self.info_subscription = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_goal_handle = None
        self.navigation_status = "IDLE"
        self.last_goal_sent_time = 0.0
        
        # Visual Tracking Stats
        self.prev_time = 0.0
        self.fps = 0.0
        self.tracker_mode = "Mono Est"
        self.fused_distance = 0.0
        self.gesture_active = False

    def scan_callback(self, msg):
        self.latest_scan = msg

    def camera_info_callback(self, msg):
        # Read camera parameters from topic if available
        self.f_x = msg.k[0]
        self.f_y = msg.k[4]
        self.c_x = msg.k[2]
        self.c_y = msg.k[5]

    def send_navigation_goal(self, pose_goal):
        if self.nav_goal_handle is not None:
            self.get_logger().info("Canceling previous navigation target...")
            self.nav_goal_handle.cancel_goal_async()
            
        self.get_logger().info("Dispatching goal target to Nav2...")
        self.navigation_status = "PLANNING"
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 '/navigate_to_pose' Action Server is offline!")
            self.navigation_status = "FAILED"
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_goal
        
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation target REJECTED by Nav2.")
            self.navigation_status = "FAILED"
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
        if status == 4: # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Robot successfully reached target position!")
            self.navigation_status = "ARRIVED"
        else:
            self.get_logger().warn(f"Navigation failed (status={status})")
            self.navigation_status = "FAILED"
        self.nav_goal_handle = None

    def image_callback(self, msg):
        start_time = time.time()
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        annotated_img = cv_img.copy()
        orig_h, orig_w = cv_img.shape[:2]
        
        # BGR Colors
        COLOR_NEON_BLUE = (255, 191, 0)
        COLOR_NEON_PINK = (203, 192, 255)
        COLOR_EMERALD = (87, 207, 80)
        COLOR_AMBER = (0, 165, 255)
        COLOR_DARK_GRAY = (40, 40, 40)
        COLOR_WHITE = (255, 255, 255)
        COLOR_CRIMSON = (45, 0, 220)
        

        # All landmark in order are: 
        # 
        skeleton_connections = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6), 
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15),
            (12, 14), (14, 16)
        ]
        
        num_persons = 0
        self.gesture_active = False
        target_goal_pose = None
        
        if self.use_onnx:
            # Preprocess
            img_resized = cv2.resize(cv_img, (320, 320))
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img_norm = img_rgb.transpose(2, 0, 1)
            img_norm = np.expand_dims(img_norm, axis=0).astype(np.float32) / 255.0
            
            outputs = self.session.run(None, {self.input_name: img_norm})
            predictions = outputs[0][0].T
            
            mask = predictions[:, 4] > self.conf_threshold
            predictions = predictions[mask]
            
            if len(predictions) > 0:
                boxes = predictions[:, :4]
                scores = predictions[:, 4]
                kpts = predictions[:, 5:]
                
                nms_boxes = []
                for box in boxes:
                    cx, cy, w, h = box
                    x = cx - w / 2
                    y = cy - h / 2
                    nms_boxes.append([float(x), float(y), float(w), float(h)])
                    
                indices = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), self.conf_threshold, 0.45)
                
                if len(indices) > 0:
                    if isinstance(indices, np.ndarray):
                        indices = indices.flatten()
                    num_persons = len(indices)
                    
                    x_scale = orig_w / 320.0
                    y_scale = orig_h / 320.0
                    
                    # Choose the closest/most prominent person for navigation tracking
                    best_idx = indices[0]
                    
                    for count, idx in enumerate(indices):
                        box = nms_boxes[idx]
                        score = scores[idx]
                        kp = kpts[idx].reshape(17, 3)
                        
                        # Scale coordinates
                        x1 = int(box[0] * x_scale)
                        y1 = int(box[1] * y_scale)
                        w_box = int(box[2] * x_scale)
                        h_box = int(box[3] * y_scale)
                        x2 = x1 + w_box
                        y2 = y1 + h_box
                        
                        kpts_scaled = kp[:, :2] * np.array([x_scale, y_scale])
                        kpts_conf = kp[:, 2]
                        
                        # Draw bounding box brackets
                        box_w, box_h = x2 - x1, y2 - y1
                        bracket_len = int(min(box_w, box_h) * 0.15)
                        
                        cv2.line(annotated_img, (x1, y1), (x1 + bracket_len, y1), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1), (x1, y1 + bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1), (x1 + box_w - bracket_len, y1), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1), (x1 + box_w, y1 + bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1 + box_h), (x1 + bracket_len, y1 + box_h), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1 + box_h), (x1, y1 + box_h - bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1 + box_h), (x1 + box_w - bracket_len, y1 + box_h), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1 + box_h), (x1 + box_w, y1 + box_h - bracket_len), COLOR_AMBER, 2)
                        
                        # Semi-transparent overlay
                        overlay = annotated_img.copy()
                        cv2.rectangle(overlay, (x1, y1), (x2, y2), COLOR_AMBER, -1)
                        cv2.addWeighted(overlay, 0.1, annotated_img, 0.9, 0, annotated_img)
                        
                        # Display Label
                        label = f"Person: {score:.2f}"
                        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(annotated_img, (x1, y1 - lh - 8), (x1 + lw + 12, y1), COLOR_AMBER, -1)
                        cv2.putText(annotated_img, label, (x1 + 6, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1, cv2.LINE_AA)
                        
                        # Draw skeleton lines
                        for pt1_idx, pt2_idx in skeleton_connections:
                            if pt1_idx < len(kpts_scaled) and pt2_idx < len(kpts_scaled):
                                x1_k, y1_k = kpts_scaled[pt1_idx].astype(int)
                                x2_k, y2_k = kpts_scaled[pt2_idx].astype(int)
                                if kpts_conf[pt1_idx] > 0.4 and kpts_conf[pt2_idx] > 0.4:
                                    cv2.line(annotated_img, (x1_k, y1_k), (x2_k, y2_k), COLOR_NEON_BLUE, 2, cv2.LINE_AA)
                                    
                        # Draw keypoints
                        for pt_idx in range(len(kpts_scaled)):
                            xk, yk = kpts_scaled[pt_idx].astype(int)
                            if kpts_conf[pt_idx] > 0.4:
                                cv2.circle(annotated_img, (xk, yk), 4, COLOR_NEON_PINK, -1, cv2.LINE_AA)
                                cv2.circle(annotated_img, (xk, yk), 6, COLOR_WHITE, 1, cv2.LINE_AA)
                                
                        # Process tracking logic for the main target
                        if idx == best_idx:
                            # 1. Wrist Gesture Check
                            nose_y = kpts_scaled[0, 1]
                            left_wrist_y = kpts_scaled[9, 1]
                            right_wrist_y = kpts_scaled[10, 1]
                            
                            if kpts_conf[0] > 0.4:
                                if (kpts_conf[9] > 0.4 and left_wrist_y < nose_y) or (kpts_conf[10] > 0.4 and right_wrist_y < nose_y):
                                    self.gesture_active = True
                                    
                            # 2. Distance Estimation from Shoulders
                            left_shoulder = kpts_scaled[5]
                            right_shoulder = kpts_scaled[6]
                            
                            if kpts_conf[5] > 0.4 and kpts_conf[6] > 0.4:
                                dx = left_shoulder[0] - right_shoulder[0]
                                dy = left_shoulder[1] - right_shoulder[1]
                                d_pixel = np.sqrt(dx**2 + dy**2)
                                
                                if d_pixel > 5.0:
                                    Z_est = (0.4 * self.f_x) / d_pixel
                                    shoulder_center_u = (left_shoulder[0] + right_shoulder[0]) / 2.0
                                    shoulder_center_v = (left_shoulder[1] + right_shoulder[1]) / 2.0
                                    theta_est = np.arctan2(shoulder_center_u - self.c_x, self.f_x)
                                    
                                    # 3. Sensor Fusion (LiDAR Gating)
                                    d_lidar = Z_est
                                    self.tracker_mode = "Mono Est"
                                    
                                    if self.latest_scan is not None:
                                        scan = self.latest_scan
                                        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
                                        angles_norm = np.arctan2(np.sin(angles), np.cos(angles))
                                        
                                        angle_diff = np.arctan2(
                                            np.sin(angles_norm - (theta_est + self.camera_laser_yaw_offset)),
                                            np.cos(angles_norm - (theta_est + self.camera_laser_yaw_offset))
                                        )
                                        
                                        mask_angle = np.abs(angle_diff) <= (5.0 * np.pi / 180.0)
                                        valid_ranges = []
                                        for s_idx in np.where(mask_angle)[0]:
                                            r = scan.ranges[s_idx]
                                            if scan.range_min <= r <= scan.range_max:
                                                if np.abs(r - Z_est) <= 0.5:
                                                    valid_ranges.append(r)
                                                    
                                        if len(valid_ranges) > 0:
                                            d_lidar = np.median(valid_ranges)
                                            self.tracker_mode = "LiDAR Fused"
                                            
                                    self.fused_distance = d_lidar
                                    
                                    # 4. Target Projection (Camera Optical Frame)
                                    Z_cam = d_lidar
                                    X_cam = Z_cam * (shoulder_center_u - self.c_x) / self.f_x
                                    Y_cam = Z_cam * (shoulder_center_v - self.c_y) / self.f_y
                                    
                                    # 5. Transform Target to Map Frame
                                    point_cam = PointStamped()
                                    point_cam.header.frame_id = self.camera_frame_id
                                    point_cam.header.stamp = rclpy.time.Time().to_msg() # Use Time 0 (latest available) to prevent future extrapolation errors
                                    point_cam.point.x = X_cam
                                    point_cam.point.y = Y_cam
                                    point_cam.point.z = Z_cam
                                    
                                    has_valid_target = False
                                    target_x_map = 0.0
                                    target_y_map = 0.0
                                    
                                    # Try TF Lookup camera->map directly
                                    try:
                                        if self.tf_buffer.can_transform(self.map_frame_id, self.camera_frame_id, rclpy.time.Time()):
                                            point_map = self.tf_buffer.transform(point_cam, self.map_frame_id)
                                            target_x_map = point_map.point.x
                                            target_y_map = point_map.point.y
                                            has_valid_target = True
                                    except Exception as ex:
                                        self.get_logger().warn(f"Direct camera-to-map transform failed: {ex}")
                                        
                                    if not has_valid_target:
                                        # Fallback: Transform manually to base_link first, then base_link->map
                                        X_base = self.camera_mount_x + Z_cam
                                        Y_base = self.camera_mount_y - X_cam
                                        
                                        point_base = PointStamped()
                                        point_base.header.frame_id = self.base_frame_id
                                        point_base.header.stamp = rclpy.time.Time().to_msg() # Use Time 0 (latest available) to prevent future extrapolation errors
                                        point_base.point.x = X_base
                                        point_base.point.y = Y_base
                                        point_base.point.z = self.camera_mount_z - Y_cam
                                        
                                        try:
                                            if self.tf_buffer.can_transform(self.map_frame_id, self.base_frame_id, rclpy.time.Time()):
                                                point_map = self.tf_buffer.transform(point_base, self.map_frame_id)
                                                target_x_map = point_map.point.x
                                                target_y_map = point_map.point.y
                                                has_valid_target = True
                                        except Exception as tf_ex:
                                            self.get_logger().warn(f"Fallback base-to-map transform failed: {tf_ex}")
                                            
                                    # 6. Generate Safety Offset Goal Coordinates
                                    if has_valid_target:
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
                                        except Exception as tf_rob_ex:
                                            self.get_logger().warn(f"Could not retrieve robot pose: {tf_rob_ex}")
        else:
            # Fallback PyTorch Path
            results = self.model.predict(source=cv_img, conf=self.conf_threshold, verbose=False)
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                keypoints = result.keypoints
                if keypoints is not None and boxes is not None:
                    num_persons = len(boxes)
                    for i in range(num_persons):
                        box = boxes[i].xyxy[0].cpu().numpy().astype(int)
                        conf = float(boxes[i].conf[0].cpu().numpy())
                        x1, y1, x2, y2 = box
                        box_w, box_h = x2 - x1, y2 - y1
                        bracket_len = int(min(box_w, box_h) * 0.15)
                        
                        cv2.line(annotated_img, (x1, y1), (x1 + bracket_len, y1), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1), (x1, y1 + bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1), (x1 + box_w - bracket_len, y1), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1), (x1 + box_w, y1 + bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1 + box_h), (x1 + bracket_len, y1 + box_h), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1, y1 + box_h), (x1, y1 + box_h - bracket_len), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1 + box_h), (x1 + box_w - bracket_len, y1 + box_h), COLOR_AMBER, 2)
                        cv2.line(annotated_img, (x1 + box_w, y1 + box_h), (x1 + box_w, y1 + box_h - bracket_len), COLOR_AMBER, 2)
                        
                        overlay = annotated_img.copy()
                        cv2.rectangle(overlay, (x1, y1), (x2, y2), COLOR_AMBER, -1)
                        cv2.addWeighted(overlay, 0.1, annotated_img, 0.9, 0, annotated_img)
                        
                        label = f"Person: {conf:.2f}"
                        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(annotated_img, (x1, y1 - lh - 8), (x1 + lw + 12, y1), COLOR_AMBER, -1)
                        cv2.putText(annotated_img, label, (x1 + 6, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1, cv2.LINE_AA)
                        
                        kpts = keypoints.xy[i].cpu().numpy()
                        kpts_conf = keypoints.conf[i].cpu().numpy() if keypoints.conf is not None else np.ones(17)
                        
                        for pt1_idx, pt2_idx in skeleton_connections:
                            if pt1_idx < len(kpts) and pt2_idx < len(kpts):
                                x1_k, y1_k = kpts[pt1_idx].astype(int)
                                x2_k, y2_k = kpts[pt2_idx].astype(int)
                                if kpts_conf[pt1_idx] > 0.4 and kpts_conf[pt2_idx] > 0.4:
                                    cv2.line(annotated_img, (x1_k, y1_k), (x2_k, y2_k), COLOR_NEON_BLUE, 2, cv2.LINE_AA)
                                    
                        for pt_idx in range(len(kpts)):
                            xk, yk = kpts[pt_idx].astype(int)
                            if kpts_conf[pt_idx] > 0.4:
                                cv2.circle(annotated_img, (xk, yk), 4, COLOR_NEON_PINK, -1, cv2.LINE_AA)
                                cv2.circle(annotated_img, (xk, yk), 6, COLOR_WHITE, 1, cv2.LINE_AA)
                                
                        if i == 0:
                            # 1. Gesture check (fallback path)
                            nose_y = kpts[0, 1]
                            left_wrist_y = kpts[9, 1]
                            right_wrist_y = kpts[10, 1]
                            if kpts_conf[0] > 0.4:
                                if (kpts_conf[9] > 0.4 and left_wrist_y < nose_y) or (kpts_conf[10] > 0.4 and right_wrist_y < nose_y):
                                    self.gesture_active = True
                                    
        # Trigger navigation client dispatch on Gesture + Cooldown
        if self.gesture_active and target_goal_pose is not None:
            cur_time = time.time()
            if (cur_time - self.last_goal_sent_time) > 6.0:
                self.last_goal_sent_time = cur_time
                self.send_navigation_goal(target_goal_pose)
                
        # Calculate inference time & FPS
        inf_time_ms = (time.time() - start_time) * 1000.0
        current_time = time.time()
        if self.prev_time > 0.0:
            current_fps = 1.0 / (current_time - self.prev_time)
            self.fps = 0.9 * self.fps + 0.1 * current_fps
        else:
            self.fps = 1.0 / (current_time - start_time)
        self.prev_time = current_time
        
        # HUD Panel Draw (Top-Left corner, taller for extra stats)
        hud_w, hud_h = 240, 160
        hud_overlay = annotated_img.copy()
        cv2.rectangle(hud_overlay, (10, 10), (10 + hud_w, 10 + hud_h), COLOR_DARK_GRAY, -1)
        cv2.addWeighted(hud_overlay, 0.65, annotated_img, 0.35, 0, annotated_img)
        
        # Border
        cv2.rectangle(annotated_img, (10, 10), (10 + hud_w, 10 + hud_h), COLOR_EMERALD, 1, cv2.LINE_AA)
        
        # Header text
        cv2.putText(annotated_img, "YOLOv11 SENSOR FUSION", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_EMERALD, 1, cv2.LINE_AA)
        cv2.line(annotated_img, (20, 38), (10 + hud_w - 10, 38), COLOR_EMERALD, 1)
        
        backend_name = "ONNX Runtime (CPU-4T)" if self.use_onnx else "PyTorch (CPU)"
        cv2.putText(annotated_img, f"Engine: {backend_name}", (20, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(annotated_img, f"FPS: {self.fps:.1f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(annotated_img, f"Latency: {inf_time_ms:.1f} ms", (20, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        
        # Sensor fusion and gesture stats
        dist_str = f"Range: {self.fused_distance:.2f} m ({self.tracker_mode})" if num_persons > 0 else "Range: N/A"
        cv2.putText(annotated_img, dist_str, (20, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)
        
        gest_color = COLOR_EMERALD if self.gesture_active else COLOR_WHITE
        gest_str = "Gesture: TRIGGERED" if self.gesture_active else "Gesture: WAITING"
        cv2.putText(annotated_img, gest_str, (20, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.4, gest_color, 1, cv2.LINE_AA)
        
        nav_color = COLOR_EMERALD if self.navigation_status in ["ARRIVED", "EXECUTING"] else (COLOR_CRIMSON if self.navigation_status == "FAILED" else COLOR_WHITE)
        cv2.putText(annotated_img, f"Nav Status: {self.navigation_status}", (20, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.4, nav_color, 1, cv2.LINE_AA)
        cv2.putText(annotated_img, f"Detections: {num_persons}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1, cv2.LINE_AA)

        # Publish
        try:
            pub_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            pub_msg.header = msg.header
            self.publisher.publish(pub_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing failed: {e}")

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
