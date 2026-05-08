#!/usr/bin/env python3
"""Scan relay node - fixes frame_id for simulation."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelayNode(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info('Scan relay started - fixing frame_id for sim')

    def cb(self, msg):
        msg.header.frame_id = 'rplidar_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
