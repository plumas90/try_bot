#!/usr/bin/env python3

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from ultralytics import YOLO


class TargetFinderNode(Node):
    def __init__(self):
        super().__init__('target_finder_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug_image_topic', '/image_yolo_debug')
        self.declare_parameter('confidence_threshold', 0.25)

        self.image_topic = self.get_parameter('image_topic').value
        self.debug_topic = self.get_parameter('debug_image_topic').value
        self.conf_th = float(self.get_parameter('confidence_threshold').value)

        self.target_class = 'laptop'
        self.bridge = CvBridge()
        self.model_path = self.find_model()
        self.model = YOLO(str(self.model_path))

        self.create_subscription(String, '/mission/target_class', self.target_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.found_pub = self.create_publisher(Bool, '/target_found', 10)
        self.offset_pub = self.create_publisher(Float32, '/target_offset', 10)
        self.area_pub = self.create_publisher(Float32, '/target_box_area_ratio', 10)
        self.width_pub = self.create_publisher(Float32, '/target_box_width_ratio', 10)
        self.height_pub = self.create_publisher(Float32, '/target_box_height_ratio', 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)

        self.frame_count = 0

        self.get_logger().info(f'TargetFinder ready. model={self.model_path}')
        self.get_logger().info(f'image={self.image_topic} -> {self.debug_topic}')

    def find_model(self):
        root = Path.home() / 'Workspace/capstone_ws'

        candidates = [
            root / 'models/yolov8n.pt',
            root / 'src/capstone_perception/models/yolov8n.pt',
            root / 'src/capstone_perception/capstone_perception/yolov8n.pt',
        ]

        for p in candidates:
            if p.exists():
                return p

        for p in root.rglob('*.pt'):
            if 'build' in p.parts or 'install' in p.parts or 'log' in p.parts:
                continue
            return p

        raise FileNotFoundError(
            'YOLO model not found. Put yolov8n.pt in '
            '~/Workspace/capstone_ws/models/yolov8n.pt'
        )

    def target_cb(self, msg):
        target = msg.data.strip().lower()

        if target:
            if target == 'notebook':
                target = 'laptop'
            self.target_class = target

    def publish_false(self, frame=None):
        self.found_pub.publish(Bool(data=False))
        self.offset_pub.publish(Float32(data=0.0))
        self.area_pub.publish(Float32(data=0.0))
        self.width_pub.publish(Float32(data=0.0))
        self.height_pub.publish(Float32(data=0.0))

        if frame is not None:
            cv2.putText(
                frame,
                f'TARGET: {self.target_class} NOT FOUND',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(out)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image convert failed: {exc}')
            self.publish_false()
            return

        h, w = frame.shape[:2]

        try:
            results = self.model(frame, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO failed: {exc}')
            self.publish_false(frame)
            return

        best = None
        best_area = 0.0
        seen = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = result.names.get(cls_id, '').lower()

                if conf < self.conf_th:
                    continue

                seen.append(name)

                if name != self.target_class:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                bw = max(0.0, x2 - x1)
                bh = max(0.0, y2 - y1)
                area = bw * bh

                if area > best_area:
                    best_area = area
                    best = (x1, y1, x2, y2, conf, name)

        if best is None:
            self.publish_false(frame)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'TARGET NOT FOUND: target={self.target_class}, '
                    f'seen={sorted(set(seen))}'
                )
            return

        x1, y1, x2, y2, conf, name = best
        cx = (x1 + x2) / 2.0
        bw = max(0.0, x2 - x1)
        bh = max(0.0, y2 - y1)

        offset = ((cx / float(w)) - 0.5) * 2.0
        area_ratio = (bw * bh) / float(w * h)
        width_ratio = bw / float(w)
        height_ratio = bh / float(h)

        self.found_pub.publish(Bool(data=True))
        self.offset_pub.publish(Float32(data=float(offset)))
        self.area_pub.publish(Float32(data=float(area_ratio)))
        self.width_pub.publish(Float32(data=float(width_ratio)))
        self.height_pub.publish(Float32(data=float(height_ratio)))

        cv2.rectangle(
            frame,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (0, 255, 0),
            2,
        )

        label = f'TARGET {name} {conf:.2f} off={offset:.2f} w={width_ratio:.2f}'
        cv2.putText(
            frame,
            label,
            (int(x1), max(25, int(y1) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(out)

        self.get_logger().info(
            f'TARGET FOUND: {name}, offset={offset:.2f}, '
            f'box_w={width_ratio:.2f}, area={area_ratio:.2f}',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetFinderNode()

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
