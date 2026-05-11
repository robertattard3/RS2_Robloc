#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros


class EefPosePublisher(Node):

    def __init__(self):
        super().__init__('eef_pose_publisher')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('eef_frame',  'gripper_tcp')
        self.declare_parameter('rate_hz',    10.0)

        self._base = self.get_parameter('base_frame').value
        self._eef  = self.get_parameter('eef_frame').value
        rate_hz    = self.get_parameter('rate_hz').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(PoseStamped, '/end_effector_position', 10)
        self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base, self._eef, rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base

        t = tf.transform.translation
        r = tf.transform.rotation
        msg.pose.position.x    = t.x
        msg.pose.position.y    = t.y
        msg.pose.position.z    = t.z
        msg.pose.orientation.x = r.x
        msg.pose.orientation.y = r.y
        msg.pose.orientation.z = r.z
        msg.pose.orientation.w = r.w

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EefPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
