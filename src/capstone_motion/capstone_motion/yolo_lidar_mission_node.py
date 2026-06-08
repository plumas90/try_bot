import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from ultralytics import YOLO


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class YoloLidarMissionNode(Node):
    def __init__(self):
        super().__init__('yolo_lidar_mission_node')

        self.declare_parameter('model_path', 'models/yolov8m.pt')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('mission_sequence', 'laptop')
        self.declare_parameter('return_home', True)
        self.declare_parameter('target_class', '')
        self.declare_parameter('use_ocr_next_target', True)
        self.declare_parameter('ocr_timeout_sec', 8.0)
        self.declare_parameter('home_marker_ocr_timeout_sec', 10.0)
        self.declare_parameter('max_mission_targets', 10)
        self.declare_parameter('allowed_target_classes', 'chair,laptop,backpack')
        self.declare_parameter('label_aliases', 'suitcase:backpack,handbag:backpack,couch:chair,dining table:chair')
        self.declare_parameter('initial_forward_sec', 0.0)
        self.declare_parameter('initial_forward_speed', 0.12)
        self.declare_parameter('initial_emergency_stop_distance', 0.10)
        self.declare_parameter('initial_spin_360_sec', 18.0)

        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('process_every_n_frames', 3)

        self.declare_parameter('stop_distance', 0.55)
        self.declare_parameter('obstacle_stop_distance', 0.20)
        self.declare_parameter('front_angle_deg', 18.0)
        self.declare_parameter('side_angle_min_deg', 25.0)
        self.declare_parameter('side_angle_max_deg', 75.0)
        self.declare_parameter('use_paper_stop', True)
        self.declare_parameter('paper_stop_area_ratio', 0.045)
        self.declare_parameter('paper_center_deadband', 0.28)
        self.declare_parameter('paper_angular_gain', 0.18)
        self.declare_parameter('paper_forward_speed', 0.08)
        self.declare_parameter('paper_search_timeout_sec', 8.0)
        self.declare_parameter('max_paper_search_attempts', 3)
        self.declare_parameter('max_paper_search_attempts_with_cue', 5)
        self.declare_parameter('paper_reposition_sec', 1.2)
        self.declare_parameter('paper_reposition_linear_speed', -0.07)
        self.declare_parameter('paper_reposition_angular_speed', 0.18)
        self.declare_parameter('paper_visual_reposition_forward_speed', 0.04)
        self.declare_parameter('paper_cue_min_area_ratio', 0.003)
        self.declare_parameter('paper_cue_min_black_ratio', 0.01)
        self.declare_parameter('paper_cue_memory_sec', 2.0)
        self.declare_parameter('object_front_distance', 0.85)
        self.declare_parameter('object_front_min_box_width_ratio', 0.20)
        self.declare_parameter('object_close_box_width_ratio', 0.42)
        self.declare_parameter('laptop_object_close_box_width_ratio', 0.65)
        self.declare_parameter('paper_search_angular_speed', 0.18)

        # V6: faster approach + hard target lock.
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('min_forward_speed_when_target_found', 0.15)
        self.declare_parameter('locked_blind_forward_speed', 0.13)
        self.declare_parameter('angular_gain', 0.10)
        self.declare_parameter('max_angular_speed', 0.07)
        self.declare_parameter('center_deadband', 0.35)
        self.declare_parameter('search_angular_speed', 0.35)
        self.declare_parameter('turn_sign', -1.0)
        self.declare_parameter('target_memory_sec', 99.0)
        self.declare_parameter('lock_on_first_detection', True)
        self.declare_parameter('arrive_hold_sec', 1.5)

        self.declare_parameter('return_linear_speed', 0.14)
        self.declare_parameter('return_angular_gain', 1.2)
        self.declare_parameter('return_max_angular_speed', 0.45)
        self.declare_parameter('return_stop_distance', 0.60)
        self.declare_parameter('return_waypoint_tolerance', 0.18)
        self.declare_parameter('return_heading_deadband', 0.25)
        self.declare_parameter('record_path_min_distance', 0.05)
        self.declare_parameter('final_turn_speed', 0.35)
        self.declare_parameter('final_turn_tolerance', 0.08)

        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('image_timeout_sec', 1.0)
        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('require_scan_for_motion', False)
        self.declare_parameter('allow_blind_search_rotation', True)
        self.declare_parameter('odom_timeout_sec', 1.0)

        self.bridge = CvBridge()
        self.model = self.load_model()

        self.sequence = self.parse_sequence()
        self.target_index = 0
        self.state = 'INITIAL_FORWARD' if float(self.get_parameter('initial_forward_sec').value) > 0.0 else 'SEARCH_TARGET'
        self.state_started_at = 0.0
        self.initial_forward_started = False
        self.arrive_until = 0.0
        self.ocr_until = 0.0
        self.pending_ocr_target = ''
        self.target_locked = False
        self.locked_target = ''

        self.frame_count = 0
        self.last_image_time = 0.0
        self.last_scan_time = 0.0
        self.last_odom_time = 0.0
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.target_found = False
        self.target_ever_found = False
        self.last_target_time = 0.0
        self.target_offset_x = 0.0
        self.target_label = ''
        self.target_confidence = 0.0
        self.target_box_width_ratio = 0.0
        self.target_box_area_ratio = 0.0
        self.locked_offset_x = 0.0
        self.locked_box_width_ratio = 0.0
        self.locked_last_seen_time = 0.0
        self.locked_offset_x = 0.0
        self.locked_box_width_ratio = 0.0
        self.locked_last_seen_time = 0.0
        self.seen_demo_labels = []

        self.paper_found = False
        self.paper_offset_x = 0.0
        self.paper_area_ratio = 0.0
        self.paper_search_attempts = 0
        self.paper_search_started_at = 0.0
        self.paper_reposition_until = 0.0
        self.paper_reposition_turn_sign = 1.0
        self.paper_cue_found = False
        self.paper_cue_offset_x = 0.0
        self.paper_cue_area_ratio = 0.0
        self.paper_cue_score = 0.0
        self.paper_cue_last_seen_time = 0.0

        self.odom_ready = False
        self.home_x = None
        self.home_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_path = []
        self.return_path = []
        self.return_waypoint_index = 0
        self.final_turn_target_yaw = None
        self.image_topic = self.param_str('image_topic')
        self.scan_topic = self.param_str('scan_topic')

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, self.param_str('cmd_vel_topic'), 10)
        self.target_found_pub = self.create_publisher(Bool, '/target_found', 10)
        self.target_offset_pub = self.create_publisher(Float32, '/target_offset_x', 10)
        self.front_distance_pub = self.create_publisher(Float32, '/front_distance', 10)
        self.debug_text_pub = self.create_publisher(String, '/motion_debug_text', 10)
        self.debug_image_pub = self.create_publisher(Image, '/target_debug_image', 10)
        self.current_target_pub = self.create_publisher(String, '/mission/current_target', 10)
        self.mission_state_pub = self.create_publisher(String, '/mission/state', 10)
        self.ocr_enable_pub = self.create_publisher(Bool, '/mission/ocr_enable', 10)
        self.alert_pub = self.create_publisher(String, '/mission/alert', 10)

        self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, sensor_qos)
        self.create_subscription(Odometry, self.param_str('odom_topic'), self.odom_cb, sensor_qos)
        self.create_subscription(String, '/mission/ocr_text', self.ocr_text_cb, 10)

        publish_hz = float(self.get_parameter('publish_hz').value)
        self.timer = self.create_timer(1.0 / publish_hz, self.control_cb)

        self.get_logger().info('V6 yolo_lidar_mission_node started')
        self.get_logger().info(f'mission_sequence = {self.sequence}')
        self.get_logger().info('Flow: YOLO + LiDAR approach -> OCR next target -> odom path return home')

    def param_str(self, name):
        return str(self.get_parameter(name).value)

    def parse_sequence(self):
        override = self.param_str('target_class').strip().lower()
        if override:
            return [override]
        raw = self.param_str('mission_sequence')
        parts = [p.strip().lower() for p in raw.split(',') if p.strip()]
        if not parts:
            return ['laptop']
        return parts

    def allowed_targets(self):
        raw = self.param_str('allowed_target_classes')
        return {p.strip().lower() for p in raw.split(',') if p.strip()}

    def normalize_target_text(self, text):
        target = text.strip().lower()
        if target == 'notebook':
            target = 'laptop'
        return target

    def label_alias_map(self):
        aliases = {}
        raw = self.param_str('label_aliases')
        for item in raw.split(','):
            if ':' not in item:
                continue
            source, target = item.split(':', 1)
            source = source.strip().lower()
            target = self.normalize_target_text(target)
            if source and target:
                aliases[source] = target
        return aliases

    def normalize_detection_label(self, label):
        label = label.strip().lower()
        return self.label_alias_map().get(label, label)

    def remaining_targets(self):
        visited = set(self.sequence[:self.target_index])
        active = self.active_target_name()
        remaining = {target for target in self.allowed_targets() if target not in visited}
        if active:
            remaining.add(active)
        return remaining

    def current_target(self):
        if self.target_index >= len(self.sequence):
            return ''
        return self.sequence[self.target_index]

    def lock_current_target(self):
        target = self.current_target()
        if not target:
            return
        if not self.target_locked:
            self.target_locked = True
            self.locked_target = target
            self.state = 'APPROACH_LOCKED'
            self.publish_debug(f'LOCKED: {target}')
            self.get_logger().warn(f'LOCKED TARGET: {target}')

    def clear_target_lock(self):
        self.target_locked = False
        self.locked_target = ''

    def active_target_name(self):
        if self.target_locked and self.locked_target:
            return self.locked_target
        return self.current_target()

    def load_model(self):
        raw_path = Path(self.param_str('model_path')).expanduser()
        candidates = [raw_path]

        if not raw_path.is_absolute():
            workspace_root = Path(__file__).resolve().parents[3]
            candidates.extend([
                Path.cwd() / raw_path,
                workspace_root / raw_path,
                Path.home() / 'Workspace/capstone_ws' / raw_path,
            ])

        model_path = next((p for p in candidates if p.exists()), None)
        if model_path is None:
            checked = ', '.join(str(p) for p in candidates)
            self.get_logger().error(f'Model file not found. Checked: {checked}')
            return None
        model = YOLO(str(model_path))
        try:
            model.to('cuda')
            self.get_logger().info('YOLO running on CUDA')
        except Exception as e:
            self.get_logger().warn(f'CUDA unavailable, using CPU: {e}')
        return model

    def image_cb(self, msg):
        self.last_image_time = time.time()
        self.frame_count += 1
        n = int(self.get_parameter('process_every_n_frames').value)
        if n < 1:
            n = 1
        if self.frame_count % n != 0:
            return
        if self.model is None:
            self.set_target(False, 0.0, '', 0.0, 0.0, 0.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'image convert failed: {exc}')
            self.set_target(False, 0.0, '', 0.0, 0.0, 0.0)
            return

        paper_found, paper_offset_x, paper_area_ratio, cue_found, cue_offset_x, cue_area_ratio, cue_score = self.find_paper(frame)
        self.paper_found = paper_found
        self.paper_offset_x = paper_offset_x
        self.paper_area_ratio = paper_area_ratio
        self.paper_cue_found = cue_found
        if cue_found:
            self.paper_cue_offset_x = cue_offset_x
            self.paper_cue_area_ratio = cue_area_ratio
            self.paper_cue_score = cue_score
            self.paper_cue_last_seen_time = time.time()

        found, offset_x, label, conf, width_ratio, area_ratio, debug_frame = self.detect_target(frame)
        self.draw_paper_debug(debug_frame)
        self.set_target(found, offset_x, label, conf, width_ratio, area_ratio)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception:
            pass

    def detect_target(self, frame):
        target_class = self.active_target_name()
        conf_threshold = float(self.get_parameter('confidence_threshold').value)
        h, w = frame.shape[:2]
        image_center_x = w / 2.0
        debug_frame = frame.copy()
        candidates = []
        seen_labels = []
        self.seen_demo_labels = []

        if not target_class or self.state in ('ARRIVE_HOLD', 'READ_NEXT_TARGET', 'READ_HOME_MARKER', 'FINAL_TURN_180', 'RETURN_HOME', 'DONE'):
            cv2.putText(debug_frame, f'MISSION {self.state}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            return False, 0.0, '', 0.0, 0.0, 0.0, debug_frame

        result = self.model(frame, verbose=False, iou=0.45)[0]
        for box in result.boxes:
            cls_id = int(box.cls[0])
            raw_label = self.model.names[cls_id].lower()
            label = self.normalize_detection_label(raw_label)
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            seen_labels.append(label)
            if label not in self.allowed_targets():
                continue
            if label in self.remaining_targets() and conf >= conf_threshold:
                self.seen_demo_labels.append(label)

            color = (255, 0, 0)
            thickness = 1
            if label == target_class and conf >= conf_threshold:
                cx = (x1 + x2) / 2.0
                area = (x2 - x1) * (y2 - y1)
                candidates.append((-area, abs(cx - image_center_x), cx, label, conf, x1, y1, x2, y2))
                color = (0, 255, 255)
                thickness = 2

            cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
            cv2.putText(debug_frame, f'{raw_label}->{label} {conf:.2f}', (int(x1), max(20, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if not candidates:
            labels_text = ','.join(sorted(set(seen_labels)))[:80]
            cv2.putText(debug_frame, f'TARGET {target_class}: NOT FOUND', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(debug_frame, f'seen: {labels_text}', (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return False, 0.0, '', 0.0, 0.0, 0.0, debug_frame

        _, _, cx, label, conf, x1, y1, x2, y2 = min(candidates)
        offset_x = (cx - image_center_x) / image_center_x
        width_ratio = max(0.0, x2 - x1) / float(w)
        area_ratio = max(0.0, x2 - x1) * max(0.0, y2 - y1) / float(w * h)
        cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
        cv2.putText(debug_frame, f'TARGET {label} {conf:.2f} off={offset_x:.2f} w={width_ratio:.2f}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(debug_frame, f'STEP {self.target_index + 1}/{len(self.sequence)}', (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return True, float(offset_x), label, float(conf), float(width_ratio), float(area_ratio), debug_frame

    def _get_paper_contours_hsv(self, frame):
        # Primary: HSV white detection — best for finding white paper against any background
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Wide white range: low saturation, high brightness
        mask = cv2.inRange(hsv, (0, 0, 120), (180, 100, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return list(contours)

    def _get_paper_contours_adaptive(self, frame):
        # Fallback: adaptive threshold intersected with white HSV mask
        # Prevents laptop keys/screen edges from being detected as paper
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 100), (180, 120, 255))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=15, C=4,
        )
        combined = cv2.bitwise_and(binary, white_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [c for c in contours if cv2.contourArea(c) > 500]

    def find_paper(self, frame):
        height, width = frame.shape[:2]
        image_area = float(width * height)
        image_center_x = width / 2.0
        contours = self._get_paper_contours_hsv(frame)
        if not contours:
            contours = self._get_paper_contours_adaptive(frame)

        best = None
        best_score = 0.0
        best_cue = None
        best_cue_score = 0.0
        cue_min_area = float(self.get_parameter('paper_cue_min_area_ratio').value)
        cue_min_black = float(self.get_parameter('paper_cue_min_black_ratio').value)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            area_ratio = (w * h) / image_area
            if area_ratio < cue_min_area or area_ratio > 0.45:
                continue
            aspect_ratio = w / float(h)
            if aspect_ratio < 0.5 or aspect_ratio > 5.0:
                continue
            roi = frame[y:y + h, x:x + w]
            black_ratio = self.black_text_ratio(roi)
            score = area_ratio + black_ratio * 0.8
            if black_ratio >= cue_min_black and score > best_cue_score:
                best_cue_score = score
                best_cue = (x, y, w, h, area_ratio, score)
            if area_ratio >= 0.01 and 0.7 <= aspect_ratio <= 4.0 and score > best_score:
                best_score = score
                best = (x, y, w, h, area_ratio)

        cue_found = best_cue is not None
        cue_offset_x = 0.0
        cue_area_ratio = 0.0
        cue_score = 0.0
        if cue_found:
            cx, cy, cw, ch, cue_area_ratio, cue_score = best_cue
            cue_offset_x = ((cx + cw / 2.0) - image_center_x) / image_center_x

        if best is None:
            return False, 0.0, 0.0, cue_found, float(cue_offset_x), float(cue_area_ratio), float(cue_score)

        x, y, w, h, area_ratio = best
        offset_x = ((x + w / 2.0) - image_center_x) / image_center_x
        return True, float(offset_x), float(area_ratio), cue_found, float(cue_offset_x), float(cue_area_ratio), float(cue_score)

    def black_text_ratio(self, roi):
        if roi.size == 0:
            return 0.0
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, black = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
        return cv2.countNonZero(black) / float(roi.shape[0] * roi.shape[1])

    def draw_paper_debug(self, frame):
        if not self.paper_found:
            if self.paper_cue_found:
                cv2.putText(frame, f'PAPER CUE off={self.paper_cue_offset_x:.2f} area={self.paper_cue_area_ratio:.3f}', (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            else:
                cv2.putText(frame, 'PAPER: not found', (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            return
        cv2.putText(frame, f'PAPER off={self.paper_offset_x:.2f} area={self.paper_area_ratio:.3f}', (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def scan_cb(self, msg):
        front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        side_min = math.radians(float(self.get_parameter('side_angle_min_deg').value))
        side_max = math.radians(float(self.get_parameter('side_angle_max_deg').value))
        front_values = []
        left_values = []
        right_values = []

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) <= front_angle:
                front_values.append(float(distance))
            elif side_min <= angle <= side_max:
                left_values.append(float(distance))
            elif -side_max <= angle <= -side_min:
                right_values.append(float(distance))

        self.last_scan_time = time.time()
        if front_values:
            self.front_distance = min(front_values)
            out = Float32()
            out.data = float(self.front_distance)
            self.front_distance_pub.publish(out)
        else:
            self.front_distance = None
        self.left_distance = min(left_values) if left_values else None
        self.right_distance = min(right_values) if right_values else None

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_x = float(p.x)
        self.current_y = float(p.y)
        self.current_yaw = yaw_from_quaternion(q)
        self.last_odom_time = time.time()

        if not self.odom_ready:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.odom_ready = True
            self.odom_path = [(self.current_x, self.current_y)]
            self.get_logger().info(f'Home odom saved: x={self.home_x:.3f}, y={self.home_y:.3f}')
            return

        self.record_odom_path()

    def record_odom_path(self):
        if self.state == 'RETURN_HOME':
            return
        if not self.odom_path:
            self.odom_path.append((self.current_x, self.current_y))
            return
        min_dist = float(self.get_parameter('record_path_min_distance').value)
        last_x, last_y = self.odom_path[-1]
        if math.hypot(self.current_x - last_x, self.current_y - last_y) >= min_dist:
            self.odom_path.append((self.current_x, self.current_y))

    def ocr_text_cb(self, msg):
        target = self.normalize_target_text(msg.data)
        if not target:
            return
        if self.state == 'READ_HOME_MARKER':
            if target == 'home':
                self.publish_debug('HOME MARKER READ: final 180 turn')
                self.start_final_turn()
            return
        if self.state != 'READ_NEXT_TARGET':
            return
        if target == 'home':
            self.pending_ocr_target = target
            self.publish_debug('OCR NEXT: HOME')
            return
        if target not in self.allowed_targets():
            self.get_logger().warn(f'OCR ignored unknown target: {target}')
            return
        self.pending_ocr_target = target
        self.publish_debug(f'OCR NEXT TARGET: {target}')

    def set_target(self, found, offset_x, label, conf, width_ratio=0.0, area_ratio=0.0):
        self.target_found = bool(found)
        if self.target_found:
            self.target_ever_found = True
            self.last_target_time = time.time()
            self.target_offset_x = float(offset_x)
            self.target_label = str(label)
            self.target_confidence = float(conf)
            self.target_box_width_ratio = float(width_ratio)
            self.target_box_area_ratio = float(area_ratio)
            self.locked_offset_x = self.target_offset_x
            self.locked_box_width_ratio = self.target_box_width_ratio
            self.locked_last_seen_time = self.last_target_time
            if self.state in ('SEARCH_TARGET', 'APPROACH_LOCKED', 'ALIGN_PAPER') and bool(self.get_parameter('lock_on_first_detection').value):
                self.lock_current_target()

        msg = Bool()
        msg.data = self.target_found
        self.target_found_pub.publish(msg)

        off = Float32()
        off.data = self.target_offset_x
        self.target_offset_pub.publish(off)

    def control_cb(self):
        self.publish_state_topics()
        if self.state not in ('READ_NEXT_TARGET', 'READ_HOME_MARKER'):
            self.set_ocr_enabled(False)
        now = time.time()
        if self.state == 'DONE':
            self.publish_stop('DONE: mission complete')
            return

        if self.state == 'INITIAL_FORWARD':
            self.control_initial_forward(now)
            return
        if self.state == 'INITIAL_SPIN_360':
            self.control_initial_spin_360(now)
            return

        if not self.check_basic_inputs(now):
            return

        if self.state == 'ARRIVE_HOLD':
            self.control_arrive_hold(now)
            return
        if self.state == 'READ_NEXT_TARGET':
            self.control_read_next_target(now)
            return
        if self.state == 'READ_HOME_MARKER':
            self.control_read_home_marker(now)
            return
        if self.state == 'FINAL_TURN_180':
            self.control_final_turn(now)
            return
        if self.state == 'RETURN_HOME':
            self.control_return_home(now)
            return
        if self.state == 'REPOSITION_FOR_PAPER':
            self.control_reposition_for_paper(now)
            return
        if self.state == 'ALIGN_PAPER':
            self.control_align_paper()
            return

        if self.target_locked:
            self.control_locked_target()
            return
        if self.target_found:
            self.lock_current_target()
            self.control_locked_target()
            return
        self.control_search()

    def check_basic_inputs(self, now):
        image_timeout = float(self.get_parameter('image_timeout_sec').value)
        scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        if now - self.last_image_time > image_timeout:
            if self.state == 'SEARCH_TARGET' and bool(self.get_parameter('allow_blind_search_rotation').value):
                return True
            self.publish_stop(
                f'STOP: no image on {self.image_topic}. '
                f'Image topics={self.visible_image_topics()}'
            )
            return False
        if not bool(self.get_parameter('require_scan_for_motion').value):
            return True
        if now - self.last_scan_time > scan_timeout:
            self.publish_stop(f'STOP: no scan on {self.scan_topic}')
            return False
        if self.front_distance is None:
            self.publish_stop('STOP: no valid front lidar')
            return False
        return True

    def visible_image_topics(self):
        topics = []
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' in types:
                topics.append(name)
        return topics

    def is_recently_seen(self, now):
        memory_sec = float(self.get_parameter('target_memory_sec').value)
        return self.target_ever_found and (now - self.last_target_time <= memory_sec)

    def is_front_too_close(self):
        # Vision-first: box filling >90% of frame = dangerously close
        box_width = max(self.target_box_width_ratio, self.locked_box_width_ratio)
        if box_width >= 0.90:
            return True
        # LiDAR as secondary check only when data is available
        dist = float(self.get_parameter('obstacle_stop_distance').value)
        return self.front_distance is not None and self.front_distance <= dist

    def is_target_distance_reached(self):
        dist = float(self.get_parameter('stop_distance').value)
        return self.front_distance is not None and self.front_distance <= dist

    def is_object_front_reached(self):
        # Vision-primary: stop based on bounding box size
        width_ratio = self.object_close_box_width_threshold()
        box_width = max(self.target_box_width_ratio, self.locked_box_width_ratio)
        if box_width >= width_ratio:
            return True
        # LiDAR as secondary when available
        dist = float(self.get_parameter('object_front_distance').value)
        min_width_ratio = float(self.get_parameter('object_front_min_box_width_ratio').value)
        if self.front_distance is not None and self.front_distance <= dist and box_width >= min_width_ratio:
            return True
        return False

    def object_close_box_width_threshold(self):
        if self.active_target_name() == 'laptop':
            return float(self.get_parameter('laptop_object_close_box_width_ratio').value)
        return float(self.get_parameter('object_close_box_width_ratio').value)

    def is_paper_reading_pose_reached(self):
        if not bool(self.get_parameter('use_paper_stop').value):
            return False
        if not self.paper_found:
            return False
        min_area = float(self.get_parameter('paper_stop_area_ratio').value)
        deadband = float(self.get_parameter('paper_center_deadband').value)
        return self.paper_area_ratio >= min_area and abs(self.paper_offset_x) <= deadband

    def control_initial_forward(self, now):
        if not self.initial_forward_started:
            self.initial_forward_started = True
            self.state_started_at = now

        duration = float(self.get_parameter('initial_forward_sec').value)
        if now - self.state_started_at >= duration:
            self.state = 'INITIAL_SPIN_360'
            self.state_started_at = now
            self.reset_target_tracking()
            self.control_initial_spin_360(now)
            return

        cmd = Twist()
        cmd.linear.x = float(self.get_parameter('initial_forward_speed').value)
        cmd.angular.z = 0.0
        self.publish_cmd(cmd, f'INITIAL_FORWARD: t={now - self.state_started_at:.1f}/{duration:.1f}, front={self.front_text()}')

    def control_initial_spin_360(self, now):
        duration = float(self.get_parameter('initial_spin_360_sec').value)
        if now - self.state_started_at >= duration:
            self.state = 'SEARCH_TARGET'
            self.state_started_at = now
            self.control_search()
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -abs(float(self.get_parameter('search_angular_speed').value))
        self.publish_cmd(cmd, f'INITIAL_SPIN_360: t={now - self.state_started_at:.1f}/{duration:.1f}, target={self.current_target()}, seen={self.target_found}, w={cmd.angular.z:.3f}')

    def control_search(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -abs(float(self.get_parameter('search_angular_speed').value))
        target = self.current_target()
        seen = ','.join(sorted(set(self.seen_demo_labels))) or 'none'
        self.publish_cmd(cmd, f'SEARCH: target={target}, remaining={sorted(self.remaining_targets())}, seen_demo={seen}, front={self.front_text()}')

    def control_locked_target(self):
        target = self.active_target_name()
        if self.is_front_too_close():
            self.publish_obstacle_avoidance(f'APPROACH_OBJECT_FRONT target={target}')
            return
        if self.is_object_front_reached():
            self.start_align_paper(reset_attempts=True)
            self.publish_debug(
                f'OBJECT_FRONT: {target}, now align paper, front={self.front_text()}, '
                f'box_w={self.target_box_width_ratio:.3f}, '
                f'close_w={self.object_close_box_width_threshold():.3f}, '
                f'paper_found={self.paper_found}'
            )
            self.control_align_paper()
            return

        cmd = Twist()
        if self.target_found:
            cmd.linear.x = max(
                float(self.get_parameter('min_forward_speed_when_target_found').value),
                float(self.get_parameter('linear_speed').value),
            )
            cmd.angular.z = self.compute_small_turn()
            self.publish_cmd(cmd, f'APPROACH_OBJECT_FRONT: target={target}, offset={self.target_offset_x:.3f}, box_w={self.target_box_width_ratio:.3f}, front={self.front_text()}, paper_seen={self.paper_found}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}')
            return

        cmd.linear.x = float(self.get_parameter('locked_blind_forward_speed').value)
        cmd.angular.z = self.compute_turn_for_offset(self.locked_offset_x) * 0.7
        age = time.time() - self.locked_last_seen_time if self.locked_last_seen_time else 0.0
        self.publish_cmd(cmd, f'APPROACH_LOCKED_BLIND: target={target}, locked_offset={self.locked_offset_x:.3f}, locked_box_w={self.locked_box_width_ratio:.3f}, lost_age={age:.1f}, front={self.front_text()}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}')

    def control_align_paper(self):
        target = self.active_target_name()
        now = time.time()
        if self.is_front_too_close():
            self.publish_obstacle_avoidance(f'ALIGN_PAPER target={target}')
            return
        if self.is_paper_reading_pose_reached():
            self.mark_target_arrived(f'ARRIVED(PAPER_CENTER): {target}, paper_area={self.paper_area_ratio:.3f}, paper_offset={self.paper_offset_x:.3f}, front={self.front_text()}')
            return

        cmd = Twist()
        if self.paper_found:
            deadband = float(self.get_parameter('paper_center_deadband').value)
            if abs(self.paper_offset_x) > deadband:
                cmd.linear.x = 0.0
                cmd.angular.z = self.compute_paper_turn()
                self.publish_cmd(cmd, f'CENTER_PAPER: target={target}, paper_offset={self.paper_offset_x:.3f}, paper_area={self.paper_area_ratio:.3f}, front={self.front_text()}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}')
                return

            cmd.linear.x = float(self.get_parameter('paper_forward_speed').value)
            cmd.angular.z = 0.0
            self.publish_cmd(cmd, f'APPROACH_PAPER_CENTER: target={target}, paper_offset={self.paper_offset_x:.3f}, paper_area={self.paper_area_ratio:.3f}, front={self.front_text()}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}')
            return

        if self.has_recent_paper_cue(now):
            timeout = float(self.get_parameter('paper_search_timeout_sec').value)
            if now - self.paper_search_started_at >= timeout:
                if self.handle_paper_attempt_failed('paper cue not confirmed'):
                    return

            deadband = float(self.get_parameter('paper_center_deadband').value)
            cmd.linear.x = 0.0
            if abs(self.paper_cue_offset_x) <= deadband and not self.is_front_too_close():
                cmd.linear.x = float(self.get_parameter('paper_visual_reposition_forward_speed').value)
            cmd.angular.z = self.compute_paper_offset_turn(self.paper_cue_offset_x)
            if cmd.angular.z == 0.0 and cmd.linear.x == 0.0:
                cmd.angular.z = float(self.get_parameter('paper_search_angular_speed').value) * 0.5
            self.publish_cmd(
                cmd,
                f'TRACK_PAPER_CUE: target={target}, attempt={self.paper_search_attempts + 1}/'
                f'{self.max_paper_search_attempts(now)}, '
                f'cue_offset={self.paper_cue_offset_x:.3f}, cue_area={self.paper_cue_area_ratio:.3f}, '
                f'front={self.front_text()}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}'
            )
            return

        cmd.linear.x = 0.0
        cmd.angular.z = float(self.get_parameter('paper_search_angular_speed').value)
        if self.paper_search_started_at <= 0.0:
            self.paper_search_started_at = now

        timeout = float(self.get_parameter('paper_search_timeout_sec').value)
        if now - self.paper_search_started_at >= timeout:
            if self.handle_paper_attempt_failed('paper not found'):
                return

        attempt = self.paper_search_attempts + 1
        max_attempts = self.max_paper_search_attempts(now)
        self.publish_cmd(cmd, f'FIND_ATTACHED_PAPER: target={target}, attempt={attempt}/{max_attempts}, front={self.front_text()}, box_w={self.target_box_width_ratio:.3f}')

    def start_align_paper(self, reset_attempts=False):
        self.state = 'ALIGN_PAPER'
        self.paper_search_started_at = time.time()
        if reset_attempts:
            self.paper_search_attempts = 0

    def handle_paper_attempt_failed(self, reason):
        self.paper_search_attempts += 1
        max_attempts = self.max_paper_search_attempts(time.time())
        if self.paper_search_attempts >= max_attempts:
            self.set_ocr_enabled(False)
            self.pending_ocr_target = ''
            self.start_return_home(f'paper search failed {self.paper_search_attempts}/{max_attempts}: {reason}')
            return True

        self.start_paper_reposition(reason)
        return True

    def start_paper_reposition(self, reason):
        self.state = 'REPOSITION_FOR_PAPER'
        duration = float(self.get_parameter('paper_reposition_sec').value)
        self.paper_reposition_until = time.time() + duration
        direction = 1.0 if self.paper_search_attempts % 2 == 1 else -1.0
        self.paper_reposition_turn_sign = direction
        max_attempts = self.max_paper_search_attempts(time.time())
        self.publish_debug(
            f'PAPER REPOSITION {self.paper_search_attempts}/{max_attempts}: {reason}'
        )

    def control_reposition_for_paper(self, now):
        if now >= self.paper_reposition_until:
            self.start_align_paper(reset_attempts=False)
            self.publish_debug('PAPER REPOSITION DONE: retry paper search')
            return

        cmd = Twist()
        if self.has_recent_paper_cue(now):
            deadband = float(self.get_parameter('paper_center_deadband').value)
            cmd.linear.x = 0.0
            if abs(self.paper_cue_offset_x) <= deadband and not self.is_front_too_close():
                cmd.linear.x = float(self.get_parameter('paper_visual_reposition_forward_speed').value)
            cmd.angular.z = self.compute_paper_offset_turn(self.paper_cue_offset_x)
        else:
            cmd.linear.x = float(self.get_parameter('paper_reposition_linear_speed').value)
            cmd.angular.z = (
                self.paper_reposition_turn_sign
                * abs(float(self.get_parameter('paper_reposition_angular_speed').value))
            )
        max_attempts = self.max_paper_search_attempts(now)
        self.publish_cmd(
            cmd,
            f'REPOSITION_FOR_PAPER: attempt={self.paper_search_attempts + 1}/{max_attempts}, '
            f'cue={self.paper_cue_text(now)}, front={self.front_text()}, '
            f'v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}'
        )

    def has_recent_paper_cue(self, now):
        memory_sec = float(self.get_parameter('paper_cue_memory_sec').value)
        return self.paper_cue_last_seen_time > 0.0 and now - self.paper_cue_last_seen_time <= memory_sec

    def max_paper_search_attempts(self, now):
        base_attempts = int(self.get_parameter('max_paper_search_attempts').value)
        if self.has_recent_paper_cue(now):
            cue_attempts = int(self.get_parameter('max_paper_search_attempts_with_cue').value)
            return max(base_attempts, cue_attempts)
        return base_attempts

    def paper_cue_text(self, now):
        if not self.has_recent_paper_cue(now):
            return 'none'
        return f'off={self.paper_cue_offset_x:.3f},area={self.paper_cue_area_ratio:.3f}'

    def publish_obstacle_avoidance(self, context):
        cmd = Twist()
        cmd.linear.x = 0.0
        turn_speed = max(float(self.get_parameter('search_angular_speed').value), 0.18)
        left = self.left_distance if self.left_distance is not None else 0.0
        right = self.right_distance if self.right_distance is not None else 0.0
        cmd.angular.z = turn_speed if left >= right else -turn_speed
        self.publish_cmd(cmd, f'AVOID_OBSTACLE: {context}, min_gap=0.20m, front={self.front_text()}, left={self.distance_text(self.left_distance)}, right={self.distance_text(self.right_distance)}, w={cmd.angular.z:.3f}')

    def mark_target_arrived(self, text):
        self.publish_stop(text)
        self.publish_alert(f'ARRIVED: {self.active_target_name()}')
        self.state = 'ARRIVE_HOLD'
        self.arrive_until = time.time() + float(self.get_parameter('arrive_hold_sec').value)

    def control_arrive_hold(self, now):
        self.publish_stop(f'HOLD: arrived {self.active_target_name()}')
        if now < self.arrive_until:
            return

        if bool(self.get_parameter('use_ocr_next_target').value):
            self.state = 'READ_NEXT_TARGET'
            self.ocr_until = now + float(self.get_parameter('ocr_timeout_sec').value)
            self.pending_ocr_target = ''
            self.set_ocr_enabled(True)
            self.publish_debug(f'OCR READ: arrived {self.active_target_name()}')
            return

        self.advance_to_next_sequence_target()

    def advance_to_next_sequence_target(self):
        self.target_index += 1
        self.reset_target_tracking()
        if self.target_index < len(self.sequence):
            self.state = 'SEARCH_TARGET'
            self.publish_debug(f'NEXT TARGET: {self.current_target()}')
            return

        if bool(self.get_parameter('return_home').value):
            self.start_return_home('all targets visited')
        else:
            self.state = 'DONE'
            self.publish_debug('DONE: all targets visited')

    def control_read_next_target(self, now):
        self.publish_stop(f'OCR: reading next target from paper, current={self.active_target_name()}')

        if self.pending_ocr_target:
            next_target = self.pending_ocr_target
            self.set_ocr_enabled(False)
            self.pending_ocr_target = ''
            self.target_index += 1
            self.reset_target_tracking()

            if next_target == 'home':
                self.start_return_home('OCR requested HOME')
                return

            max_targets = int(self.get_parameter('max_mission_targets').value)
            if self.target_index >= max_targets:
                self.start_return_home('Max mission targets reached')
                return

            if self.target_index < len(self.sequence):
                self.sequence[self.target_index] = next_target
            else:
                self.sequence.append(next_target)

            self.state = 'SEARCH_TARGET'
            self.publish_debug(f'NEXT TARGET FROM OCR: {self.current_target()}')
            return

        if now <= self.ocr_until:
            return

        self.set_ocr_enabled(False)
        self.pending_ocr_target = ''
        if self.handle_paper_attempt_failed('OCR timeout'):
            return

    def control_read_home_marker(self, now):
        self.publish_stop('HOME: reading start marker before final turn')
        self.set_ocr_enabled(True)
        if now <= self.ocr_until:
            return
        self.publish_debug('HOME MARKER OCR TIMEOUT: final 180 turn fallback')
        self.start_final_turn()

    def reset_target_tracking(self):
        self.clear_target_lock()
        self.target_found = False
        self.target_ever_found = False
        self.target_offset_x = 0.0
        self.target_label = ''
        self.target_confidence = 0.0
        self.target_box_width_ratio = 0.0
        self.target_box_area_ratio = 0.0
        self.paper_search_attempts = 0
        self.paper_search_started_at = 0.0
        self.paper_reposition_until = 0.0
        self.paper_reposition_turn_sign = 1.0

    def start_return_home(self, reason):
        if bool(self.get_parameter('return_home').value):
            self.build_return_path()
            self.state = 'RETURN_HOME'
            self.publish_debug(f'NEXT: RETURN_HOME ({reason})')
        else:
            self.state = 'DONE'
            self.publish_debug(f'DONE: {reason}')

    def build_return_path(self):
        self.return_path = list(reversed(self.odom_path))
        if self.home_x is not None and self.home_y is not None:
            if not self.return_path:
                self.return_path = [(self.home_x, self.home_y)]
            elif math.hypot(self.return_path[-1][0] - self.home_x, self.return_path[-1][1] - self.home_y) > 0.01:
                self.return_path.append((self.home_x, self.home_y))
        self.return_waypoint_index = 0

    def control_return_home(self, now):
        odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        if not self.odom_ready or now - self.last_odom_time > odom_timeout:
            self.publish_stop('STOP: no odom for return home')
            return
        if self.is_front_too_close():
            self.publish_stop(f'STOP: obstacle while returning, front={self.front_distance:.3f}')
            return

        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        dist = math.hypot(dx, dy)
        stop_dist = float(self.get_parameter('return_stop_distance').value)
        if dist <= stop_dist:
            self.state = 'READ_HOME_MARKER'
            self.ocr_until = now + float(self.get_parameter('home_marker_ocr_timeout_sec').value)
            self.set_ocr_enabled(True)
            self.publish_alert('RETURNED_HOME')
            self.publish_stop(f'HOME: returned by odom, dist={dist:.3f}, reading HOME marker')
            return

        waypoint_x, waypoint_y = self.next_return_waypoint()
        desired_yaw = math.atan2(waypoint_y - self.current_y, waypoint_x - self.current_x)
        yaw_error = normalize_angle(desired_yaw - self.current_yaw)
        cmd = Twist()
        cmd.angular.z = clamp(
            float(self.get_parameter('return_angular_gain').value) * yaw_error,
            -float(self.get_parameter('return_max_angular_speed').value),
            float(self.get_parameter('return_max_angular_speed').value),
        )
        heading_deadband = float(self.get_parameter('return_heading_deadband').value)
        if abs(yaw_error) <= heading_deadband:
            cmd.linear.x = float(self.get_parameter('return_linear_speed').value)
        else:
            cmd.linear.x = 0.0
        self.publish_cmd(cmd, f'RETURN_HOME: dist={dist:.3f}, wp={self.return_waypoint_index + 1}/{len(self.return_path)}, yaw_error={yaw_error:.3f}, v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}')

    def start_final_turn(self):
        self.set_ocr_enabled(False)
        self.final_turn_target_yaw = normalize_angle(self.current_yaw + math.pi)
        self.state = 'FINAL_TURN_180'
        self.publish_alert('HOME_CONFIRMED')
        self.publish_debug(f'FINAL_TURN_180: target_yaw={self.final_turn_target_yaw:.3f}')

    def control_final_turn(self, now):
        odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        if not self.odom_ready or now - self.last_odom_time > odom_timeout:
            self.publish_stop('STOP: no odom for final 180 turn')
            return
        if self.final_turn_target_yaw is None:
            self.start_final_turn()
            return

        yaw_error = normalize_angle(self.final_turn_target_yaw - self.current_yaw)
        tolerance = float(self.get_parameter('final_turn_tolerance').value)
        if abs(yaw_error) <= tolerance:
            self.state = 'DONE'
            self.publish_alert('MISSION_DONE')
            self.publish_stop(f'DONE: final 180 turn complete, yaw_error={yaw_error:.3f}')
            return

        max_speed = float(self.get_parameter('final_turn_speed').value)
        cmd = Twist()
        cmd.angular.z = clamp(1.4 * yaw_error, -max_speed, max_speed)
        self.publish_cmd(cmd, f'FINAL_TURN_180: yaw_error={yaw_error:.3f}, w={cmd.angular.z:.3f}')

    def next_return_waypoint(self):
        if not self.return_path:
            self.build_return_path()
        tolerance = float(self.get_parameter('return_waypoint_tolerance').value)
        while self.return_waypoint_index < len(self.return_path) - 1:
            wx, wy = self.return_path[self.return_waypoint_index]
            if math.hypot(wx - self.current_x, wy - self.current_y) > tolerance:
                break
            self.return_waypoint_index += 1
        if self.return_waypoint_index >= len(self.return_path):
            return self.home_x, self.home_y
        return self.return_path[self.return_waypoint_index]

    def compute_small_turn(self):
        return self.compute_turn_for_offset(self.target_offset_x)

    def compute_turn_for_offset(self, offset_x):
        deadband = float(self.get_parameter('center_deadband').value)
        if abs(offset_x) <= deadband:
            return 0.0
        gain = float(self.get_parameter('angular_gain').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)
        turn_sign = float(self.get_parameter('turn_sign').value)
        return clamp(turn_sign * gain * offset_x, -max_ang, max_ang)

    def compute_paper_turn(self):
        return self.compute_paper_offset_turn(self.paper_offset_x)

    def compute_paper_offset_turn(self, offset_x):
        deadband = float(self.get_parameter('paper_center_deadband').value) * 0.5
        if abs(offset_x) <= deadband:
            return 0.0
        gain = float(self.get_parameter('paper_angular_gain').value)
        max_ang = max(
            float(self.get_parameter('max_angular_speed').value),
            float(self.get_parameter('paper_search_angular_speed').value),
        )
        turn_sign = float(self.get_parameter('turn_sign').value)
        return clamp(turn_sign * gain * offset_x, -max_ang, max_ang)

    def front_text(self):
        if self.front_distance is None:
            return 'none'
        return f'{self.front_distance:.3f}'

    def distance_text(self, value):
        if value is None:
            return 'none'
        return f'{value:.3f}'

    def publish_state_topics(self):
        target_msg = String()
        target_msg.data = self.active_target_name()
        self.current_target_pub.publish(target_msg)
        state_msg = String()
        state_msg.data = self.state
        self.mission_state_pub.publish(state_msg)

    def publish_cmd(self, cmd, text):
        self.cmd_pub.publish(cmd)
        self.publish_debug(text)
        self.get_logger().info(text, throttle_duration_sec=0.5)

    def publish_stop(self, text='STOP'):
        self.cmd_pub.publish(Twist())
        self.publish_debug(text)
        self.get_logger().info(text, throttle_duration_sec=0.5)

    def publish_debug(self, text):
        msg = String()
        msg.data = text
        self.debug_text_pub.publish(msg)

    def publish_alert(self, text):
        msg = String()
        msg.data = text
        self.alert_pub.publish(msg)
        self.get_logger().warn(f'\aALERT: {text}')

    def set_ocr_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.ocr_enable_pub.publish(msg)

    def stop_repeated(self, count=30, interval=0.02):
        for _ in range(count):
            self.cmd_pub.publish(Twist())
            time.sleep(interval)


def main(args=None):
    rclpy.init(args=args)
    node = YoloLidarMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_repeated()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
