#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class WalkController(Node):
    def __init__(self):
        super().__init__('walk_controller')
        
        # Publisher to legs position controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/legs_position_controller/commands',
            10
        )
        
        # Walking parameters
        self.frequency = 0.5  # Hz (walking speed)
        self.step_height = 0.05  # meters
        self.step_length = 0.1  # meters forward
        
        # Neutral standing pose
        self.neutral_pose = {
            'hip_pitch': -0.15,
            'hip_roll': 0.0,
            'hip_yaw': 0.0,
            'knee': 0.3,
            'ankle_pitch': -0.15,
            'ankle_roll': 0.0
        }
        
        self.time = 0.0
        self.dt = 0.02  # 50 Hz control loop
        
        # Publish at 50 Hz
        self.timer = self.create_timer(self.dt, self.publish_walking_pattern)
        
        self.get_logger().info('Walking controller started')
        self.get_logger().info(f'Frequency: {self.frequency} Hz, Step height: {self.step_height}m')

    def walking_gait(self, t, leg='left'):
        """
        Generate walking gait for one leg
        Returns joint angles for hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
        """
        phase = 2 * math.pi * self.frequency * t
        
        # Right leg is 180 degrees out of phase
        if leg == 'right':
            phase += math.pi
        
        # Normalize phase to [0, 2*pi]
        phase = phase % (2 * math.pi)
        
        # Swing phase (leg in air): 0 to pi
        # Stance phase (leg on ground): pi to 2*pi
        
        hip_pitch = self.neutral_pose['hip_pitch']
        hip_roll = self.neutral_pose['hip_roll']
        hip_yaw = self.neutral_pose['hip_yaw']
        knee = self.neutral_pose['knee']
        ankle_pitch = self.neutral_pose['ankle_pitch']
        ankle_roll = self.neutral_pose['ankle_roll']
        
        if phase < math.pi:  # Swing phase - lift leg
            swing_progress = phase / math.pi
            
            # Hip pitch: move leg forward
            hip_pitch += self.step_length * math.sin(swing_progress * math.pi)
            
            # Knee: bend more to lift foot
            knee += self.step_height * 2 * math.sin(swing_progress * math.pi)
            
            # Ankle: adjust to keep foot level
            ankle_pitch -= self.step_height * math.sin(swing_progress * math.pi)
            
        else:  # Stance phase - push back
            stance_progress = (phase - math.pi) / math.pi
            
            # Hip pitch: move leg backward (push body forward)
            hip_pitch -= self.step_length * stance_progress
        
        return [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]

    def publish_walking_pattern(self):
        """Publish joint commands for walking"""
        # Get joint angles for both legs
        left_joints = self.walking_gait(self.time, 'left')
        right_joints = self.walking_gait(self.time, 'right')
        
        # Combine: [left_leg (6 joints), right_leg (6 joints)]
        all_joints = left_joints + right_joints
        
        # Publish
        msg = Float64MultiArray()
        msg.data = all_joints
        self.publisher.publish(msg)
        
        # Increment time
        self.time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = WalkController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
