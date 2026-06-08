#!/usr/bin/env python3

import re
import time
from difflib import SequenceMatcher

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    import pytesseract
except Exception:
    pytesseract = None


class OcrReaderNode(Node):
    def __init__(self):
        super().__init__('ocr_reader_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug_image_topic', '/ocr_debug_image')
        self.declare_parameter('ocr_interval_sec', 0.7)

        self.image_topic = self.get_parameter('image_topic').value
        self.debug_topic = self.get_parameter('debug_image_topic').value
        self.ocr_interval = float(self.get_parameter('ocr_interval_sec').value)

        self.bridge = CvBridge()
        self.enabled = False
        self.last_ocr_time = 0.0
        self.last_result = ''

        self.create_subscription(Bool, '/mission/ocr_enable', self.enable_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.text_pub = self.create_publisher(String, '/mission/ocr_text', 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)

        if pytesseract is None:
            self.get_logger().error(
                'pytesseract is missing. OCR node will run, but cannot read text.'
            )
        else:
            self.get_logger().info('OCR reader ready.')

    def enable_cb(self, msg):
        next_enabled = bool(msg.data)
        if next_enabled == self.enabled:
            return

        self.enabled = next_enabled

        if self.enabled:
            self.last_result = ''
            self.get_logger().info('OCR ENABLED')
        else:
            self.get_logger().info('OCR DISABLED')

    def normalize_text(self, raw):
        text = raw.upper()
        text = re.sub(r'[^A-Z]', '', text)

        words = ['CHAIR', 'BACKPACK', 'LAPTOP', 'NOTEBOOK', 'HOME']

        for w in words:
            if w in text:
                return 'LAPTOP' if w == 'NOTEBOOK' else w

        best_word = ''
        best_score = 0.0

        for w in words:
            score = SequenceMatcher(None, text, w).ratio()
            if score > best_score:
                best_score = score
                best_word = w

        if best_score >= 0.55:
            return 'LAPTOP' if best_word == 'NOTEBOOK' else best_word

        return ''

    def find_paper_roi(self, frame):
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White paper: low saturation, high brightness.
        mask = cv2.inRange(hsv, (0, 0, 130), (180, 80, 255))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

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

            # A4 landscape or cropped sign-like paper.
            if ratio < 1.0 or ratio > 3.5:
                continue

            if area > best_area:
                best_area = area
                best = (x, y, bw, bh)

        if best is None:
            # fallback: center crop
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
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        # black letters on white paper
        _, binary = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU,
        )

        scale = 3
        binary = cv2.resize(
            binary,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        )

        return binary

    def run_ocr(self, binary):
        if pytesseract is None:
            return ''

        config = (
            '--psm 6 '
            '-c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        )

        raw = pytesseract.image_to_string(binary, config=config)
        return self.normalize_text(raw)

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
        binary = self.preprocess_for_ocr(roi)
        result = self.run_ocr(binary)

        x, y, w, h = rect

        color = (0, 255, 0) if paper_found else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        show = result if result else 'OCR...'
        cv2.putText(
            frame,
            show,
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (0, 255, 255),
            3,
        )

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(out)

        if result:
            self.last_result = result
            self.text_pub.publish(String(data=result))
            self.get_logger().info(f'OCR RESULT: {result}')
        else:
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
