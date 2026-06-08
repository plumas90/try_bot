#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ForceDemoNode(Node):
    def __init__(self):
        super().__init__('force_demo_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.step = 0
        self.step_start = time.time()

        self.sequence = [
            ('STOP', 1.0),
            ('FORWARD', 3.0),
            ('STOP', 1.5),
            ('FORWARD', 3.0),
            ('STOP', 1.5),
            ('FORWARD', 3.0),
            ('STOP', 9999.0),
        ]

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().warn('FORCE DEMO STARTED')
        self.get_logger().warn('This node only publishes /cmd_vel.')

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def publish_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.04
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def loop(self):
        now = time.time()

        if self.step >= len(self.sequence):
            self.publish_stop()
            return

        mode, duration = self.sequence[self.step]

        if now - self.step_start >= duration:
            self.step += 1
            self.step_start = now
            self.publish_stop()
            return

        if mode == 'FORWARD':
            self.publish_forward()
        else:
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = ForceDemoNode()

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
