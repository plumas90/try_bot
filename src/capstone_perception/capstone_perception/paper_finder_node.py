
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class PaperFinderNode(Node):
    def __init__(self):
        super().__init__('paper_finder_node')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('min_area_ratio', 0.015)
        self.declare_parameter('max_area_ratio', 0.75)
        self.declare_parameter('min_aspect_ratio', 1.1)
        self.declare_parameter('max_aspect_ratio', 3.2)
        self.declare_parameter('min_black_ratio', 0.01)
        self.image_topic = self.get_parameter('image_topic').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.max_area_ratio = self.get_parameter('max_area_ratio').value
        self.min_aspect_ratio = self.get_parameter('min_aspect_ratio').value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').value
        self.min_black_ratio = self.get_parameter('min_black_ratio').value
        self.bridge = CvBridge()
        self.paper_found_pub = self.create_publisher(Bool, '/paper_found', 10)
        self.paper_offset_pub = self.create_publisher(Float32, '/paper_offset_x', 10)
        self.paper_area_pub = self.create_publisher(Float32, '/paper_area', 10)
        self.debug_image_pub = self.create_publisher(Image, '/paper_debug_image', 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.get_logger().info('paper_finder_node started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        found, offset_x, area_ratio, debug_frame = self.find_paper(frame)
        self.publish_result(found, offset_x, area_ratio)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

    def find_paper(self, frame):
        height, width = frame.shape[:2]
        image_area = float(width * height)
        image_center_x = width / 2.0
        debug_frame = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 80, 255]))
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = 0.0
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            area_ratio = (w * h) / image_area
            if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
                continue
            aspect_ratio = w / float(h)
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue
            roi = frame[y:y+h, x:x+w]
            black_ratio = self.get_black_ratio(roi)
            if black_ratio < self.min_black_ratio:
                continue
            score = area_ratio + black_ratio
            if score > best_score:
                best_score = score
                best = (x, y, w, h, area_ratio, black_ratio)
        if best is None:
            cv2.putText(debug_frame, 'paper not found', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            return False, 0.0, 0.0, debug_frame
        x, y, w, h, area_ratio, black_ratio = best
        paper_center_x = x + w / 2.0
        offset_x = (paper_center_x - image_center_x) / image_center_x
        cv2.rectangle(debug_frame, (x,y), (x+w,y+h), (0,255,0), 3)
        cv2.putText(debug_frame, f'paper offset={offset_x:.2f} area={area_ratio:.2f} black={black_ratio:.2f}', (x, max(30, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
        return True, float(offset_x), float(area_ratio), debug_frame

    def get_black_ratio(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, black_mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        black_pixels = cv2.countNonZero(black_mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        return 0.0 if total_pixels == 0 else black_pixels / float(total_pixels)

    def publish_result(self, found, offset_x, area_ratio):
        f = Bool(); f.data = bool(found); self.paper_found_pub.publish(f)
        o = Float32(); o.data = float(offset_x) if found else 0.0; self.paper_offset_pub.publish(o)
        a = Float32(); a.data = float(area_ratio) if found else 0.0; self.paper_area_pub.publish(a)


def main(args=None):
    rclpy.init(args=args)
    node = PaperFinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
