#!/usr/bin/env python3
"""
ROS2 YOLO Traffic Detector Node with CUDA support.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
import json
import yaml
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from .traffic_classes import ID2NAME, is_light, get_class_name
from .geometry_utils import estimate_distance_meters, calculate_detection_score


class CategoryState:
    """State manager for temporal filtering and EMA smoothing."""
    
    def __init__(self, is_light_category: bool, min_visible_frames: int, config: dict):
        self.is_light_category = is_light_category
        self.min_visible_frames = min_visible_frames
        self.config = config
        
        # State variables
        self.active = None  # (name, cx_norm, cy_norm)
        self.active_score = None
        self.active_conf_ema = None
        self.active_distance = None  # aktif tespitin metrik mesafesi

        self.candidate = None
        self.candidate_streak = 0
        
        # For light color switching
        self.pending_switch = None  # (target_name, counter)
        self.smoothed_best_score = None

    def _similar_position(self, a, b, tolerance=0.15):
        """Check if two detections are at similar positions."""
        if a is None or b is None:
            return False
        if a[0] != b[0]:  # Different class
            return False
        return (abs(a[1] - b[1]) < tolerance) and (abs(a[2] - b[2]) < tolerance)

    def update(self, best_tuple, best_score, best_conf, best_distance, frame_has_detections):
        """Update state with new frame data."""
        ema_conf = self.config['ema']['alpha_confidence']
        ema_score = self.config['ema']['alpha_score']

        # Reset if no detections in frame
        if not frame_has_detections:
            self._reset_state()
            return

        # Update smoothed score
        if best_score is not None:
            if self.smoothed_best_score is None:
                self.smoothed_best_score = best_score
            else:
                self.smoothed_best_score = (ema_score * best_score +
                                          (1 - ema_score) * self.smoothed_best_score)

        # Handle color switching for traffic lights
        if (self.is_light_category and best_tuple is not None and
            self.active is not None and best_tuple[0] != self.active[0]):
            self._handle_color_switch(best_tuple, best_conf, best_distance, ema_conf)
            return

        # Temporal filtering
        if self.active is None:
            self._handle_no_active(best_tuple, best_conf, best_distance)
        else:
            self._handle_active_present(best_tuple, best_conf, best_distance, ema_conf)

    def _reset_state(self):
        """Reset all state variables."""
        self.active = None
        self.active_score = None
        self.active_conf_ema = None
        self.active_distance = None
        self.candidate = None
        self.candidate_streak = 0
        self.pending_switch = None
        self.smoothed_best_score = None

    def _handle_color_switch(self, best_tuple, best_conf, best_distance, ema_conf):
        """Handle traffic light color switching."""
        curr_conf = self.active_conf_ema or (best_conf or 0.0)
        conf_gain = (best_conf or 0.0) - curr_conf
        boost_threshold = self.config['temporal']['class_switch_confidence_boost']

        if conf_gain >= boost_threshold:
            # Immediate switch for high confidence gain
            self._activate_detection(best_tuple, best_conf, best_distance, ema_conf)
            self.pending_switch = None
        else:
            # Gradual switch with delay
            switch_delay = self.config['temporal']['class_switch_delay']
            if (self.pending_switch is None or
                self.pending_switch[0] != best_tuple[0]):
                self.pending_switch = (best_tuple[0], 1)
            else:
                self.pending_switch = (best_tuple[0], self.pending_switch[1] + 1)

            if self.pending_switch[1] >= switch_delay:
                self._activate_detection(best_tuple, best_conf, best_distance, ema_conf)
                self.pending_switch = None

    def _handle_no_active(self, best_tuple, best_conf, best_distance):
        """Handle case when no active detection."""
        if (self.candidate is None or
            not self._similar_position(self.candidate, best_tuple) or
            (best_tuple and self.candidate[0] != best_tuple[0])):
            self.candidate = best_tuple
            self.candidate_streak = 1 if best_tuple is not None else 0
        else:
            self.candidate_streak += 1

        if (self.candidate_streak >= self.min_visible_frames and
            self.candidate is not None):
            self.active = self.candidate
            self.active_score = self.smoothed_best_score
            self.active_conf_ema = best_conf
            self.active_distance = best_distance

    def _handle_active_present(self, best_tuple, best_conf, best_distance, ema_conf):
        """Handle case when active detection exists."""
        if (best_tuple is not None and
            self.active[0] == best_tuple[0] and
            self._similar_position(self.active, best_tuple)):
            # Update existing active detection
            if self.active_conf_ema is not None and best_conf is not None:
                self.active_conf_ema = (ema_conf * best_conf +
                                      (1 - ema_conf) * self.active_conf_ema)
            self.active_score = self.smoothed_best_score
            self.active_distance = best_distance
            self.candidate = None
            self.candidate_streak = 0
        else:
            # New candidate detected
            self._handle_new_candidate(best_tuple, best_conf, best_distance, ema_conf)

    def _handle_new_candidate(self, best_tuple, best_conf, best_distance, ema_conf):
        """Handle new candidate detection."""
        if (best_tuple is not None and
            (self.candidate is None or
             not self._similar_position(self.candidate, best_tuple) or
             self.candidate[0] != best_tuple[0])):
            self.candidate = best_tuple
            self.candidate_streak = 1
        elif best_tuple is not None:
            self.candidate_streak += 1

        if (self.candidate_streak >= self.min_visible_frames and
            self.candidate is not None):
            self._activate_detection(self.candidate, best_conf, best_distance, ema_conf)

    def _activate_detection(self, detection, confidence, distance, ema_conf):
        """Activate a new detection."""
        self.active = detection
        self.active_score = self.smoothed_best_score
        self.active_distance = distance
        if confidence is not None:
            self.active_conf_ema = (ema_conf * confidence +
                                  (1 - ema_conf) * (self.active_conf_ema or confidence))
        self.candidate = None
        self.candidate_streak = 0


class YOLODetectorNode(Node):
    """ROS2 node for YOLO-based traffic detection."""

    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Load configuration
        self.config = self._load_config()

        # Mesafe kalibrasyonu: CameraInfo'dan fx okunana kadar config focal fallback
        self.dynamic_focal = None

        # Initialize YOLO model with CUDA
        self._initialize_model()

        # Initialize ROS components
        self._initialize_ros()
        
        # Initialize state managers
        self._initialize_states()
        
        # Initialize utilities
        self.bridge = CvBridge()
        self.fps_ema = None
        self.last_time = time.time()
        self.frame_counter = 0
        
        self.get_logger().info("YOLO Traffic Detector Node initialized")
        self.get_logger().info(f"Using device: {self.device}")

    def _load_config(self):
        """Load configuration from ROS2 parameters."""
        try:
            # Declare parameters with default values
            self._declare_parameters()
            
            # Get parameters
            config = {
                'model': {
                    'path': self.get_parameter('model.path').value,
                    'device': self.get_parameter('model.device').value
                },
                'camera': {
                    'topic': self.get_parameter('camera.topic').value,
                    'info_topic': self.get_parameter('camera.info_topic').value,
                    'focal_length_px': self.get_parameter('camera.focal_length_px').value,
                    'image_width': self.get_parameter('camera.image_width').value,
                    'image_height': self.get_parameter('camera.image_height').value
                },
                'thresholds': {
                    'confidence_light': self.get_parameter('thresholds.confidence_light').value,
                    'confidence_sign': self.get_parameter('thresholds.confidence_sign').value
                },
                'scoring': {
                    'weight_confidence': self.get_parameter('scoring.weight_confidence').value,
                    'weight_distance': self.get_parameter('scoring.weight_distance').value,
                    'weight_priority': self.get_parameter('scoring.weight_priority').value,
                    'weight_off_axis': self.get_parameter('scoring.weight_off_axis').value
                },
                'distance': {
                    'min_meters': self.get_parameter('distance.min_meters').value,
                    'max_meters': self.get_parameter('distance.max_meters').value
                },
                'temporal': {
                    'min_visible_frames_light': self.get_parameter('temporal.min_visible_frames_light').value,
                    'min_visible_frames_sign': self.get_parameter('temporal.min_visible_frames_sign').value,
                    'class_switch_delay': self.get_parameter('temporal.class_switch_delay').value,
                    'class_switch_confidence_boost': self.get_parameter('temporal.class_switch_confidence_boost').value
                },
                'ema': {
                    'alpha_confidence': self.get_parameter('ema.alpha_confidence').value,
                    'alpha_score': self.get_parameter('ema.alpha_score').value,
                    'alpha_fps': self.get_parameter('ema.alpha_fps').value
                },
                'processing': {
                    'max_fps': self.get_parameter('processing.max_fps').value,
                    'imgsz': self.get_parameter('processing.imgsz').value,
                    'publish_debug_image': self.get_parameter('processing.publish_debug_image').value,
                    'ignore_classes': self.get_parameter('processing.ignore_classes').value
                },
                'publishers': {
                    'detections_topic': self.get_parameter('publishers.detections_topic').value,
                    'debug_image_topic': self.get_parameter('publishers.debug_image_topic').value,
                    'active_light_topic': self.get_parameter('publishers.active_light_topic').value,
                    'active_sign_topic': self.get_parameter('publishers.active_sign_topic').value,
                    'light_distance_topic': self.get_parameter('publishers.light_distance_topic').value,
                    'sign_distance_topic': self.get_parameter('publishers.sign_distance_topic').value
                }
            }
            
            self.get_logger().info("Loaded configuration from ROS2 parameters")
            return config
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            # Return default config
            return self._get_default_config()

    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values."""
        # Model parameters
        self.declare_parameter('model.path', 'models/best.pt')
        self.declare_parameter('model.device', 'cuda:0')
        
        # Camera parameters
        self.declare_parameter('camera.topic', '/zed/zed_node/rgb_raw/image_raw_color')
        # CameraInfo: fx (k[0]) buradan otomatik okunur → mesafe kalibrasyonu
        # çözünürlükten bağımsız doğru olur. Gelmezse focal_length_px fallback.
        self.declare_parameter('camera.info_topic', '/zed/zed_node/rgb_raw/camera_info')
        self.declare_parameter('camera.focal_length_px', 528.2849731445312)
        self.declare_parameter('camera.image_width', 1280)
        self.declare_parameter('camera.image_height', 720)
        
        # Threshold parameters
        self.declare_parameter('thresholds.confidence_light', 0.60)
        self.declare_parameter('thresholds.confidence_sign', 0.50)
        
        # Scoring parameters
        self.declare_parameter('scoring.weight_confidence', 0.40)
        self.declare_parameter('scoring.weight_distance', 0.45)
        self.declare_parameter('scoring.weight_priority', 0.30)
        self.declare_parameter('scoring.weight_off_axis', 0.20)
        
        # Distance parameters
        self.declare_parameter('distance.min_meters', 1.0)
        self.declare_parameter('distance.max_meters', 40.0)
        
        # Temporal parameters
        self.declare_parameter('temporal.min_visible_frames_light', 3)
        self.declare_parameter('temporal.min_visible_frames_sign', 3)
        self.declare_parameter('temporal.class_switch_delay', 2)
        self.declare_parameter('temporal.class_switch_confidence_boost', 0.20)
        
        # EMA parameters
        self.declare_parameter('ema.alpha_confidence', 0.6)
        self.declare_parameter('ema.alpha_score', 0.5)
        self.declare_parameter('ema.alpha_fps', 0.3)
        
        # Processing parameters
        self.declare_parameter('processing.max_fps', 30.0)
        # imgsz: YOLO çıkarım çözünürlüğü — 1280 uzak/küçük tabelaları daha iyi yakalar
        self.declare_parameter('processing.imgsz', 1280)
        self.declare_parameter('processing.publish_debug_image', True)
        self.declare_parameter('processing.ignore_classes', ['yellow_light'])
        
        # Publisher parameters
        self.declare_parameter('publishers.detections_topic', '/yolo_detections')
        self.declare_parameter('publishers.debug_image_topic', '/yolo_debug_image')
        self.declare_parameter('publishers.active_light_topic', '/active_traffic_light')
        self.declare_parameter('publishers.active_sign_topic', '/active_traffic_sign')
        self.declare_parameter('publishers.light_distance_topic', '/active_traffic_light_distance')
        self.declare_parameter('publishers.sign_distance_topic', '/active_traffic_sign_distance')

    def _get_default_config(self):
        """Get default configuration."""
        return {
            'model': {
                'path': 'models/best.pt',
                'device': 'cuda:0'
            },
            'camera': {
                'topic': '/zed/zed_node/rgb_raw/image_raw_color',
                'info_topic': '/zed/zed_node/rgb_raw/camera_info',
                'focal_length_px': 528.2849731445312,
                'image_width': 1280,
                'image_height': 720
            },
            'thresholds': {
                'confidence_light': 0.60,
                'confidence_sign': 0.50
            },
            'scoring': {
                'weight_confidence': 0.40,
                'weight_distance': 0.45,
                'weight_priority': 0.30,
                'weight_off_axis': 0.20
            },
            'distance': {
                'min_meters': 1.0,
                'max_meters': 40.0
            },
            'temporal': {
                'min_visible_frames_light': 3,
                'min_visible_frames_sign': 3,
                'class_switch_delay': 2,
                'class_switch_confidence_boost': 0.20
            },
            'ema': {
                'alpha_confidence': 0.6,
                'alpha_score': 0.5,
                'alpha_fps': 0.3
            },
            'processing': {
                'max_fps': 30.0,
                'imgsz': 1280,
                'publish_debug_image': True,
                'ignore_classes': ['yellow_light']
            },
            'publishers': {
                'detections_topic': '/yolo_detections',
                'debug_image_topic': '/yolo_debug_image',
                'active_light_topic': '/active_traffic_light',
                'active_sign_topic': '/active_traffic_sign',
                'light_distance_topic': '/active_traffic_light_distance',
                'sign_distance_topic': '/active_traffic_sign_distance'
            }
        }

    def _initialize_model(self):
        """Initialize YOLO model with CUDA support."""
        try:
            # Get model path
            package_dir = get_package_share_directory('yolo_traffic_detector')
            model_path = os.path.join(package_dir, self.config['model']['path'])
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"Model file not found: {model_path}")
                raise FileNotFoundError(f"Model file not found: {model_path}")
            
            # Check CUDA availability
            device_config = self.config['model']['device']
            if device_config.startswith('cuda') and not torch.cuda.is_available():
                self.get_logger().warning("CUDA not available, falling back to CPU")
                self.device = 'cpu'
            else:
                self.device = device_config
            
            # Load model
            self.model = YOLO(model_path)
            self.model.to(self.device)
            
            self.get_logger().info(f"Model loaded successfully from: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize model: {e}")
            raise

    def _initialize_ros(self):
        """Initialize ROS publishers and subscribers."""
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.config['camera']['topic'],
            self.image_callback,
            10
        )

        # CameraInfo aboneliği — fx (k[0]) mesafe kalibrasyonu için otomatik okunur
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.config['camera']['info_topic'],
            self.camera_info_callback,
            10
        )

        # Publishers
        self.debug_image_pub = self.create_publisher(
            Image,
            self.config['publishers']['debug_image_topic'],
            10
        )

        self.active_light_pub = self.create_publisher(
            String,
            self.config['publishers']['active_light_topic'],
            10
        )

        self.active_sign_pub = self.create_publisher(
            String,
            self.config['publishers']['active_sign_topic'],
            10
        )

        # Mesafe yayınları (Float32, metre; tespit yoksa -1.0)
        self.light_distance_pub = self.create_publisher(
            Float32,
            self.config['publishers']['light_distance_topic'],
            10
        )
        self.sign_distance_pub = self.create_publisher(
            Float32,
            self.config['publishers']['sign_distance_topic'],
            10
        )

        # Ham per-frame tespitler (JSON dizisi). parking_safety'nin
        # parking_mission_planner ve stop_mission_planner dugumleri buradan
        # cep/durak tabelalarini okur. /active_traffic_* ile ILGISIZDIR:
        # o ikisi zamansal filtrelenmis "aktif" tekil sonuc, bu ise ham liste.
        self.detections_pub = self.create_publisher(
            String,
            self.config['publishers']['detections_topic'],
            10
        )

    def camera_info_callback(self, msg):
        """CameraInfo'dan fx (k[0]) oku → mesafe kalibrasyonu otomatik güncellenir."""
        try:
            fx = msg.k[0] if hasattr(msg, 'k') else msg.K[0]
            if fx and fx > 0:
                if self.dynamic_focal is None:
                    self.get_logger().info(f"CameraInfo focal alındı: fx={fx:.2f} px (otomatik kalibrasyon)")
                self.dynamic_focal = float(fx)
        except Exception as e:
            self.get_logger().warning(f"CameraInfo okunamadı: {e}")

    def _get_focal_length(self):
        """Mesafe kestirimi için focal: CameraInfo geldiyse dinamik, yoksa config."""
        if self.dynamic_focal is not None:
            return self.dynamic_focal
        return self.config['camera']['focal_length_px']

    def _initialize_states(self):
        """Initialize state managers for temporal filtering."""
        self.state_light = CategoryState(
            is_light_category=True,
            min_visible_frames=self.config['temporal']['min_visible_frames_light'],
            config=self.config
        )
        
        self.state_sign = CategoryState(
            is_light_category=False,
            min_visible_frames=self.config['temporal']['min_visible_frames_sign'],
            config=self.config
        )

    def _should_process_frame(self):
        """Check if frame should be processed based on FPS limit."""
        current_time = time.time()
        dt = current_time - self.last_time
        max_fps = self.config['processing']['max_fps']
        
        if dt < (1.0 / max_fps):
            return False
        
        self.last_time = current_time
        return True

    def _get_confidence_threshold(self, class_name):
        """Get confidence threshold for a class."""
        if 'light' in class_name:
            return self.config['thresholds']['confidence_light']
        else:
            return self.config['thresholds']['confidence_sign']

    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            # FPS limiting
            if not self._should_process_frame():
                return
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO inference (imgsz: uzak/küçük tabelaları daha iyi yakalar)
            results = self.model(cv_image, verbose=False, device=self.device,
                                 imgsz=self.config['processing']['imgsz'])[0]
            
            # Process detections
            self._process_detections(cv_image, results)
            
            # Update FPS
            self._update_fps()
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def _process_detections(self, image, results):
        """Process YOLO detections and update states."""
        height, width = image.shape[:2]
        ignore_classes = set(self.config['processing']['ignore_classes'])
        
        # Collect valid detections
        best_light, best_light_score, best_light_conf = None, None, None
        best_sign, best_sign_score, best_sign_conf = None, None, None
        best_light_distance = None
        best_sign_distance = None
        
        any_light_detected = False
        any_sign_detected = False

        # /yolo_detections icin ham per-frame liste (parking_safety kullanir)
        frame_detections = []

        # Process each detection
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = get_class_name(cls_id)
            
            # Skip ignored classes
            if class_name in ignore_classes:
                continue
                
            # Skip low confidence detections
            if confidence < self._get_confidence_threshold(class_name):
                continue
            
            # Estimate distance (focal: CameraInfo geldiyse dinamik, yoksa config)
            distance, d_min, d_max = estimate_distance_meters(
                class_name, x1, y1, x2, y2,
                self._get_focal_length()
            )
            
            # Calculate score
            score = calculate_detection_score(
                confidence, distance, class_name, x1, x2, width,
                self.config['scoring'],
                (self.config['distance']['min_meters'], 
                 self.config['distance']['max_meters'])
            )
            
            # Normalize coordinates
            cx_norm = (0.5 * (x1 + x2)) / width
            cy_norm = (0.5 * (y1 + y2)) / height
            detection_tuple = (class_name, cx_norm, cy_norm)

            # Ham tespiti listeye ekle (mesafe yoksa null; planner onu eler)
            frame_detections.append({
                'class': class_name,
                'conf': round(confidence, 4),
                'distance': (round(float(distance), 3)
                             if distance is not None else None),
                'cx': round(cx_norm, 4),
                'cy': round(cy_norm, 4),
            })

            # Draw detection on image
            self._draw_detection(image, x1, y1, x2, y2, class_name,
                               confidence, distance, d_min, d_max)
            
            # Update best detections
            if is_light(class_name):
                any_light_detected = True
                if best_light_score is None or score > best_light_score:
                    best_light = detection_tuple
                    best_light_score = score
                    best_light_conf = confidence
                    best_light_distance = distance
            else:
                any_sign_detected = True
                if best_sign_score is None or score > best_sign_score:
                    best_sign = detection_tuple
                    best_sign_score = score
                    best_sign_conf = confidence
                    best_sign_distance = distance
        
        # Update states (mesafe bilgisiyle)
        self.state_light.update(best_light, best_light_score,
                               best_light_conf, best_light_distance, any_light_detected)
        self.state_sign.update(best_sign, best_sign_score,
                              best_sign_conf, best_sign_distance, any_sign_detected)
        
        # Draw HUD and publish results
        self._draw_hud(image, best_light_distance, best_sign_distance)
        self._publish_results(image, frame_detections)

    def _draw_detection(self, image, x1, y1, x2, y2, class_name, 
                       confidence, distance, d_min, d_max):
        """Draw detection box and labels on image."""
        # Draw bounding box
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), 
                     (0, 255, 0), 2)
        
        # Draw class name and confidence
        cv2.putText(image, f"{class_name} {confidence:.2f}", 
                   (int(x1), int(y1) - 6),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        # Draw distance if available
        if distance is not None:
            distance_text = f"~{distance:.1f}m [{d_min:.1f}-{d_max:.1f}]"
            cv2.putText(image, distance_text,
                       (int(x1), int(y2) + 16),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 255), 1, cv2.LINE_AA)

    def _draw_hud(self, image, light_distance, sign_distance):
        """Draw heads-up display with active detections."""
        # Active traffic light
        active_light = self.state_light.active[0] if self.state_light.active else "None"
        cv2.putText(image, f"ACTIVE_LIGHT: {active_light}", (12, 28),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 230, 50), 2, cv2.LINE_AA)
        
        if self.state_light.active_conf_ema is not None:
            cv2.putText(image, f"conf(EMA): {self.state_light.active_conf_ema:.2f}", 
                       (320, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 230, 50), 2, cv2.LINE_AA)
        
        # Active traffic sign
        active_sign = self.state_sign.active[0] if self.state_sign.active else "None"
        cv2.putText(image, f"ACTIVE_SIGN : {active_sign}", (12, 56),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 200, 255), 2, cv2.LINE_AA)
        
        if self.state_sign.active_conf_ema is not None:
            cv2.putText(image, f"conf(EMA): {self.state_sign.active_conf_ema:.2f}", 
                       (320, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 200, 255), 2, cv2.LINE_AA)
        
        # Distance information
        if light_distance is not None:
            cv2.putText(image, f"light_dist~{light_distance:.1f} m", (12, 84),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 100), 2, cv2.LINE_AA)
        
        if sign_distance is not None:
            cv2.putText(image, f"sign_dist~{sign_distance:.1f} m", (12, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 100), 2, cv2.LINE_AA)
        
        # FPS
        if self.fps_ema is not None:
            cv2.putText(image, f"FPS: {self.fps_ema:.1f}", (12, 136),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

    def _cv2_to_imgmsg_bgr8(self, img):
        """cv_bridge'siz Image mesajı: Foxy cv_bridge'i pip OpenCV ile
        cv2_to_imgmsg'de KeyError(16) attığı için mesaj elle kurulur."""
        img = np.ascontiguousarray(img)
        msg = Image()
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg

    def _publish_results(self, image, frame_detections=None):
        """Publish detection results and debug image."""
        # Publish debug image if enabled
        if self.config['processing']['publish_debug_image']:
            try:
                debug_msg = self._cv2_to_imgmsg_bgr8(image)
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "camera_link"
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")
        
        # Publish active traffic light
        light_msg = String()
        light_msg.data = self.state_light.active[0] if self.state_light.active else "None"
        self.active_light_pub.publish(light_msg)
        
        # Publish active traffic sign
        sign_msg = String()
        sign_msg.data = self.state_sign.active[0] if self.state_sign.active else "None"
        self.active_sign_pub.publish(sign_msg)

        # Publish active detection distances (metre; tespit/mesafe yoksa -1.0)
        light_dist_msg = Float32()
        light_dist_msg.data = (float(self.state_light.active_distance)
                               if self.state_light.active_distance is not None else -1.0)
        self.light_distance_pub.publish(light_dist_msg)

        sign_dist_msg = Float32()
        sign_dist_msg.data = (float(self.state_sign.active_distance)
                              if self.state_sign.active_distance is not None else -1.0)
        self.sign_distance_pub.publish(sign_dist_msg)

        # Ham per-frame tespit listesi (parking_safety icin). Tespit yoksa bos
        # dizi yayinlanir ki abone taze "hicbir sey yok" bilgisi alsin.
        det_msg = String()
        det_msg.data = json.dumps(frame_detections if frame_detections else [])
        self.detections_pub.publish(det_msg)

    def _update_fps(self):
        """Update FPS calculation with EMA."""
        current_time = time.time()
        if hasattr(self, '_prev_time'):
            dt = max(1e-6, current_time - self._prev_time)
            instant_fps = 1.0 / dt
            alpha = self.config['ema']['alpha_fps']
            
            if self.fps_ema is None:
                self.fps_ema = instant_fps
            else:
                self.fps_ema = alpha * instant_fps + (1 - alpha) * self.fps_ema
        
        self._prev_time = current_time


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()