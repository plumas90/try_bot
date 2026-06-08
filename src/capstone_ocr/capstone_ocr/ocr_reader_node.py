#!/usr/bin/env python3

import difflib
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    import easyocr
except ImportError:
    easyocr = None


KNOWN_TARGETS = ['CHAIR', 'BACKPACK', 'LAPTOP', 'NOTEBOOK', 'HOME']
CANONICAL = {'NOTEBOOK': 'LAPTOP'}


class OcrReaderNode(Node):
    def __init__(self):
        super().__init__('ocr_reader_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug_image_topic', '/ocr_debug_image')
        self.declare_parameter('ocr_interval_sec', 0.5)

        image_topic = self.get_parameter('image_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self.ocr_interval = float(self.get_parameter('ocr_interval_sec').value)

        self.bridge = CvBridge()
        self.enabled = False
        self.last_ocr_time = 0.0
        self.last_result = ''
        self._prev_result = ''  # for consecutive-confirmation

        if easyocr is None:
            self.get_logger().error('easyocr not installed. Run: pip install easyocr')
            self.reader = None
        else:
            self.get_logger().info('Loading EasyOCR (English, GPU)...')
            self.reader = easyocr.Reader(['en'], gpu=True)
            # warm-up
            dummy = np.zeros((64, 256, 3), dtype=np.uint8)
            self.reader.readtext(dummy)
            self.get_logger().info('EasyOCR ready.')

        self.create_subscription(Bool, '/mission/ocr_enable', self.enable_cb, 10)
        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.text_pub = self.create_publisher(String, '/mission/ocr_text', 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

    def enable_cb(self, msg):
        next_enabled = bool(msg.data)
        if next_enabled == self.enabled:
            return
        self.enabled = next_enabled
        if self.enabled:
            self.last_result = ''
            self._prev_result = ''
            self.get_logger().info('OCR ENABLED')
        else:
            self.get_logger().info('OCR DISABLED')

    def normalize_text(self, raw: str) -> str:
        """Normalize raw OCR output to a canonical target name or empty string."""
        text = raw.upper().strip()
        alpha = ''.join(c for c in text if c.isalpha())

        if len(alpha) < 3:
            return ''

        if alpha in KNOWN_TARGETS:
            return CANONICAL.get(alpha, alpha)

        matches = difflib.get_close_matches(alpha, KNOWN_TARGETS, n=1, cutoff=0.6)
        if matches:
            return CANONICAL.get(matches[0], matches[0])

        return ''

    def find_paper_roi(self, frame):
        """Find white paper region using adaptive threshold, fallback to center crop."""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=15, C=4,
        )
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < float(w * h) * 0.015:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw <= 0 or bh <= 0:
                continue
            ratio = bw / float(bh)
            if ratio < 1.0 or ratio > 3.5:
                continue
            if area > best_area:
                best_area = area
                best = (x, y, bw, bh)

        if best is None:
            x = int(w * 0.15)
            y = int(h * 0.25)
            bw = int(w * 0.70)
            bh = int(h * 0.50)
            return frame[y:y + bh, x:x + bw], (x, y, bw, bh), False

        x, y, bw, bh = best
        pad_x = int(bw * 0.05)
        pad_y = int(bh * 0.08)
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(w, x + bw + pad_x)
        y2 = min(h, y + bh + pad_y)
        return frame[y1:y2, x1:x2], (x1, y1, x2 - x1, y2 - y1), True

    def preprocess_for_ocr(self, roi):
        """Sharpen and enhance contrast for better OCR input."""
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)
        gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        return gray

    def run_ocr(self, preprocessed) -> str:
        if self.reader is None:
            return ''
        results = self.reader.readtext(preprocessed, detail=1)
        for (_bbox, text, conf) in results:
            if conf < 0.3:
                continue
            normalized = self.normalize_text(text)
            if normalized:
                return normalized
        return ''

    def image_cb(self, msg):
        if not self.enabled:
            return
        now = time.time()
        if now - self.last_ocr_time < self.ocr_interval:
            return
        self.last_ocr_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image convert failed: {exc}')
            return

        roi, rect, paper_found = self.find_paper_roi(frame)
        preprocessed = self.preprocess_for_ocr(roi)
        result = self.run_ocr(preprocessed)

        x, y, w, h = rect
        color = (0, 255, 0) if paper_found else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        show = result if result else 'OCR...'
        cv2.putText(frame, show, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(out)

        if result:
            if result == self._prev_result:
                self.last_result = result
                self.text_pub.publish(String(data=result))
                self.get_logger().info(f'OCR RESULT (confirmed): {result}')
            else:
                self._prev_result = result
                self.get_logger().info(f'OCR candidate (waiting for confirm): {result}')
        else:
            self._prev_result = ''
            self.get_logger().info('OCR scanning...', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OcrReaderNode()
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
