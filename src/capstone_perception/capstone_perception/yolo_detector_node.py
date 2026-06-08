#!/usr/bin/env python3

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug_image_topic', '/image_yolo_debug')
        self.declare_parameter('confidence_threshold', 0.35)

        image_topic = self.get_parameter('image_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self.conf_th = float(self.get_parameter('confidence_threshold').value)

        self.model_path = self.find_model()
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')

        self.model = YOLO(str(self.model_path))
        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, debug_topic, 10)
        self.objects_pub = self.create_publisher(String, '/detected_objects', 10)

        self.create_subscription(Image, image_topic, self.image_cb, 10)

        self.frame_count = 0
        self.get_logger().info(
            f'YOLO detector ready: {image_topic} -> {debug_topic}'
        )

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

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image convert failed: {exc}')
            return

        try:
            results = self.model(frame, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        names = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_th:
                    continue

                cls_id = int(box.cls[0])
                name = result.names.get(cls_id, str(cls_id))
                names.append(f'{name}:{conf:.2f}')

        annotated = results[0].plot()
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.image_pub.publish(out)
        self.objects_pub.publish(String(data=', '.join(names)))

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'YOLO running. detected=[{", ".join(names)}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

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
