#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class YoloLockDriveNode(Node):
    def __init__(self):
        super().__init__('yolo_lock_drive_node')

        self.declare_parameter('linear_speed', 0.04)
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('max_drive_sec', 9.0)
        self.declare_parameter('hold_sec', 1.5)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.max_drive_sec = float(self.get_parameter('max_drive_sec').value)
        self.hold_sec = float(self.get_parameter('hold_sec').value)

        self.targets = ['chair', 'backpack', 'laptop']
        self.target_index = 0

        self.locked = False
        self.drive_start_time = 0.0
        self.hold_until = 0.0

        self.front_distance = float('inf')
        self.last_detected_text = ''

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_found_pub = self.create_publisher(Bool, '/target_found', 10)
        self.target_class_pub = self.create_publisher(String, '/mission/target_class', 10)

        self.create_subscription(String, '/detected_objects', self.detected_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().warn('YOLO LOCK DRIVE STARTED')
        self.get_logger().warn('YOLO ON. TARGET_FINDER OFF. ROTATION OFF.')

    def current_target(self):
        if self.target_index >= len(self.targets):
            return None
        return self.targets[self.target_index]

    def aliases_for(self, target):
        table = {
            'chair': ['chair'],
            'backpack': ['backpack', 'bag'],
            'laptop': ['laptop', 'notebook'],
        }
        return table.get(target, [target])

    def detected_cb(self, msg):
        text = msg.data.lower()
        self.last_detected_text = text

        target = self.current_target()
        if target is None:
            return

        if self.locked:
            return

        for word in self.aliases_for(target):
            if word in text:
                self.locked = True
                self.drive_start_time = time.time()
                self.get_logger().warn(f'YOLO LOCKED: {target}')
                return

    def scan_cb(self, msg):
        vals = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle) <= math.radians(18.0):
                vals.append(r)

        if vals:
            self.front_distance = min(vals)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_forward(self):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_status(self):
        target = self.current_target()

        found = Bool()
        found.data = self.locked
        self.target_found_pub.publish(found)

        cls = String()
        cls.data = target if target is not None else 'done'
        self.target_class_pub.publish(cls)

    def finish_target(self, reason):
        target = self.current_target()
        self.get_logger().warn(f'FINISH {target}: {reason}')

        self.publish_stop()

        self.target_index += 1
        self.locked = False
        self.drive_start_time = 0.0
        self.hold_until = time.time() + self.hold_sec

    def loop(self):
        now = time.time()
        target = self.current_target()

        self.publish_status()

        if target is None:
            self.publish_stop()
            return

        if now < self.hold_until:
            self.publish_stop()
            return

        if not self.locked:
            self.publish_stop()
            return

        if self.front_distance <= self.stop_distance:
            self.finish_target(f'lidar {self.front_distance:.2f}m')
            return

        if now - self.drive_start_time >= self.max_drive_sec:
            self.finish_target('timeout')
            return

        self.publish_forward()


def main(args=None):
    rclpy.init(args=args)
    node = YoloLockDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
