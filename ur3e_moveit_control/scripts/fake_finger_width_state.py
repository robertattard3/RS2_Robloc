#!/usr/bin/env python3
"""
Publishes a fake finger_width joint state so MoveIt's planning scene monitor
has a complete robot state when the OnRobot hardware interface is not connected
(e.g. running against URSim without a real gripper attached).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeFingerWidthState(Node):
    def __init__(self):
        super().__init__('fake_finger_width_state')
        self.declare_parameter('finger_width', 0.05)
        self.pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer_ = self.create_timer(0.02, self.publish)  # 50 Hz

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_width']
        msg.position = [self.get_parameter('finger_width').value]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.pub_.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeFingerWidthState())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
