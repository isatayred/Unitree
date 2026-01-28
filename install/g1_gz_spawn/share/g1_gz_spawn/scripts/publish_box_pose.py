#!/usr/bin/env python3
"""Publish a static box pose to /box_pose so other nodes can consume it.

Usage: publish_box_pose.py --x <x> --y <y> --z <z>
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse


class BoxPosePublisher(Node):
    def __init__(self, x, y, z):
        super().__init__('box_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, 'box_pose', 10)
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.position.x = float(x)
        self.pose.pose.position.y = float(y)
        self.pose.pose.position.z = float(z)
        # publish periodically so consumers started later still receive it
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        from time import time
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pose)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', required=True)
    parser.add_argument('--y', required=True)
    parser.add_argument('--z', required=True)
    args = parser.parse_args()

    rclpy.init()
    node = BoxPosePublisher(args.x, args.y, args.z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
