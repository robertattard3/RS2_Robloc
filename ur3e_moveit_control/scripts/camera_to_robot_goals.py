#!/usr/bin/env python3
"""
camera_to_robot_goals.py

Subscribes to a PoseArray published in the camera's frame and keeps the
latest pose cached.  When a trigger arrives on /start_pick_place, the
latest pose is transformed into the robot's planning frame (base_link by
default) and published once to /robot_goal_poses.

Parameters (set via --ros-args -p <name>:=<value>):
  input_topic   (string, default: /cube_poses)
  target_frame  (string, default: base_link)
  tf_timeout    (float,  default: 1.0)

Topics:
  Subscribed:  <input_topic>      — geometry_msgs/PoseArray in camera frame
               /start_pick_place  — std_msgs/Bool  (True = trigger)
  Published:   /robot_goal_poses  — geometry_msgs/PoseArray in target frame
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transform support

from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool


class CameraToRobotGoals(Node):

    def __init__(self):
        super().__init__('camera_to_robot_goals')

        self.declare_parameter('input_topic',  '/cube_poses')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('tf_timeout',   1.0)

        input_topic       = self.get_parameter('input_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        tf_timeout_s      = self.get_parameter('tf_timeout').value
        self.tf_timeout   = Duration(seconds=tf_timeout_s)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._latest_msg        = None   # most recent PoseArray from camera
        self._waiting_for_pose  = False  # True after Start pressed, until next pose arrives

        self.publisher_ = self.create_publisher(PoseArray, '/robot_goal_poses', 10)

        self.create_subscription(PoseArray, input_topic, self._pose_callback, 10)
        self.create_subscription(Bool, '/start_pick_place', self._start_callback, 10)

        self.get_logger().info(
            f"Ready. Caching poses from '{input_topic}', "
            f"waiting for trigger on '/start_pick_place'."
        )

    def _pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return
        self._latest_msg = msg
        if self._waiting_for_pose:
            self._waiting_for_pose = False
            self._publish_latest()

    def _start_callback(self, msg: Bool):
        if not msg.data:
            return
        # Discard any pre-existing cached pose and arm for the next fresh detection
        self._latest_msg       = None
        self._waiting_for_pose = True
        self.get_logger().info(
            "Start triggered — waiting for next cube pose from camera."
        )

    def _publish_latest(self):
        msg          = self._latest_msg
        source_frame = msg.header.frame_id

        if not source_frame:
            self.get_logger().warn("Received pose has empty frame_id — ignoring.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, msg.header.stamp, self.tf_timeout
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup with message stamp failed: {e}. Retrying with latest."
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, rclpy.time.Time(), self.tf_timeout
                )
            except Exception as e2:
                self.get_logger().error(f"TF lookup failed: {e2}. Dropping.")
                return

        out = PoseArray()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame

        for pose in msg.poses:
            ps_in        = PoseStamped()
            ps_in.header = msg.header
            ps_in.pose   = pose

            ps_out = tf2_geometry_msgs.do_transform_pose_stamped(ps_in, transform)

            # Camera is nearly horizontal so depth-derived z is unreliable.
            # Cube always rests on the table — clamp to known height and flat orientation.
            ps_out.pose.position.z    = 0.025
            ps_out.pose.orientation.x = 0.0
            ps_out.pose.orientation.y = 0.0
            ps_out.pose.orientation.z = 0.0
            ps_out.pose.orientation.w = 1.0

            out.poses.append(ps_out.pose)

        self.publisher_.publish(out)
        self.get_logger().info(
            f"Published {len(out.poses)} pose(s) from '{source_frame}' "
            f"to '{self.target_frame}' on '/robot_goal_poses'."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraToRobotGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
