import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time


class PosePublisherNode(Node):
    """Helper node that publishes Pose messages."""

    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(Pose, '/tool_pose', 10)

    def publish_pose(self, x: float, y: float, z: float):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        # Leave orientation at default (identity quaternion)
        msg.orientation.w = 1.0
        self.publisher.publish(msg)


class PoseListenerNode(Node):
    """Helper node that receives and stores Pose messages."""

    def __init__(self):
        super().__init__('pose_listener')
        self.received_poses = []
        self.subscription = self.create_subscription(
            Pose,
            '/tool_pose',
            self._callback,
            10
        )

    def _callback(self, msg):
        self.received_poses.append({
            'x': msg.position.x,
            'y': msg.position.y,
            'z': msg.position.z,
        })


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(node, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


class TestPoseDisplay:

    def test_pose_received_correctly(self, ros_context):
        """Verify a published pose is received with correct XYZ values."""
        publisher_node = PosePublisherNode()
        listener_node = PoseListenerNode()

        time.sleep(0.3)

        publisher_node.publish_pose(1.23, 4.56, 7.89)
        spin_for(listener_node, duration=1.0)

        assert len(listener_node.received_poses) > 0, \
            "No messages received on /tool_pose"

        last = listener_node.received_poses[-1]
        assert abs(last['x'] - 1.23) < 1e-3, f"X mismatch: {last['x']}"
        assert abs(last['y'] - 4.56) < 1e-3, f"Y mismatch: {last['y']}"
        assert abs(last['z'] - 7.89) < 1e-3, f"Z mismatch: {last['z']}"

        publisher_node.destroy_node()
        listener_node.destroy_node()

    def test_pose_negative_values(self, ros_context):
        """Verify negative coordinate values are handled correctly."""
        publisher_node = PosePublisherNode()
        listener_node = PoseListenerNode()

        time.sleep(0.3)

        publisher_node.publish_pose(-0.5, -1.0, -2.75)
        spin_for(listener_node, duration=1.0)

        assert len(listener_node.received_poses) > 0
        last = listener_node.received_poses[-1]
        assert abs(last['x'] - (-0.5)) < 1e-3
        assert abs(last['y'] - (-1.0)) < 1e-3
        assert abs(last['z'] - (-2.75)) < 1e-3

        publisher_node.destroy_node()
        listener_node.destroy_node()

    def test_pose_zero_values(self, ros_context):
        """Verify zero/origin pose is handled correctly."""
        publisher_node = PosePublisherNode()
        listener_node = PoseListenerNode()

        time.sleep(0.3)

        publisher_node.publish_pose(0.0, 0.0, 0.0)
        spin_for(listener_node, duration=1.0)

        assert len(listener_node.received_poses) > 0
        last = listener_node.received_poses[-1]
        assert abs(last['x']) < 1e-3
        assert abs(last['y']) < 1e-3
        assert abs(last['z']) < 1e-3

        publisher_node.destroy_node()
        listener_node.destroy_node()

    def test_pose_sequential_updates(self, ros_context):
        """Verify that rapid sequential pose updates are all received."""
        publisher_node = PosePublisherNode()
        listener_node = PoseListenerNode()

        time.sleep(0.3)

        poses = [(i * 0.1, i * 0.2, i * 0.3) for i in range(5)]
        for x, y, z in poses:
            publisher_node.publish_pose(x, y, z)
            time.sleep(0.05)

        spin_for(listener_node, duration=1.5)

        assert len(listener_node.received_poses) >= 5, \
            f"Expected at least 5 poses, got {len(listener_node.received_poses)}"

        publisher_node.destroy_node()
        listener_node.destroy_node()