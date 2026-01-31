#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StandingPosePublisher(Node):
    def __init__(self):
        super().__init__('standing_pose_publisher')
        
        # Publisher to legs position controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/legs_position_controller/commands',
            10
        )
        
        # Standing pose for all 12 leg joints (6 per leg)
        # Left leg: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
        # Right leg: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
        # Values tuned for stable standing
        self.standing_positions = [
            -0.15,  # left_hip_pitch (slight forward lean)
            0.0,    # left_hip_roll (neutral)
            0.0,    # left_hip_yaw (neutral)
            0.3,    # left_knee (bent for support)
            -0.15,  # left_ankle_pitch (balance)
            0.0,    # left_ankle_roll (neutral)
            -0.15,  # right_hip_pitch (matching left)
            0.0,    # right_hip_roll (neutral)
            0.0,    # right_hip_yaw (neutral)
            0.3,    # right_knee (bent for support)
            -0.15,  # right_ankle_pitch (balance)
            0.0,    # right_ankle_roll (neutral)
        ]
        
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_pose)
        
        self.get_logger().info('Standing pose publisher started')
        self.get_logger().info(f'Target positions: {self.standing_positions}')

    def publish_pose(self):
        msg = Float64MultiArray()
        msg.data = self.standing_positions
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandingPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
