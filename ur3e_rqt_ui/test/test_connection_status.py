import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time


class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('connection_status_test_publisher')
        self.publisher = self.create_publisher(Pose, '/tool_pose', 10)
        self.publishing = True
        self.received_timestamps = []

    def publish_pose(self):
        msg = Pose()
        msg.position.x = 1.0
        msg.position.y = 1.0
        msg.position.z = 1.0
        msg.orientation.w = 1.0
        self.publisher.publish(msg)
        self.received_timestamps.append(time.time())


class PoseListenerNode(Node):
    def __init__(self):
        super().__init__('connection_status_test_listener')
        self.last_msg_time = None
        self.create_subscription(Pose, '/tool_pose', self._cb, 10)

    def _cb(self, msg):
        self.last_msg_time = time.time()

    def is_connected(self, timeout=1.0):
        """Mirror the plugin's connection logic."""
        if self.last_msg_time is None:
            return False
        return (time.time() - self.last_msg_time) < timeout


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(node, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


class TestConnectionStatus:

    def test_connected_when_publishing(self, ros_context):
        """Indicator should show CONNECTED when messages arrive within timeout."""
        pub = PosePublisherNode()
        listener = PoseListenerNode()
        time.sleep(0.3)

        # Publish steadily for 1 second
        for _ in range(10):
            pub.publish_pose()
            spin_for(listener, duration=0.1)

        assert listener.is_connected(), \
            "Expected CONNECTED state while messages are being published"

        pub.destroy_node()
        listener.destroy_node()

    def test_no_signal_when_publishing_stops(self, ros_context):
        """Indicator should show NO SIGNAL after messages stop for > 1 second."""
        pub = PosePublisherNode()
        listener = PoseListenerNode()
        time.sleep(0.3)

        # Publish briefly to establish connection
        for _ in range(5):
            pub.publish_pose()
            spin_for(listener, duration=0.1)

        assert listener.is_connected(), \
            "Expected CONNECTED state before stopping"

        # Stop publishing and wait for timeout
        print("\nStopping publisher — waiting 1.5s for NO SIGNAL...")
        time.sleep(1.5)
        spin_for(listener, duration=0.1)

        assert not listener.is_connected(), \
            "Expected NO SIGNAL state after 1.5s without messages"

        pub.destroy_node()
        listener.destroy_node()

    def test_reconnects_after_signal_loss(self, ros_context):
        """Indicator should return to CONNECTED after signal is restored."""
        pub = PosePublisherNode()
        listener = PoseListenerNode()
        time.sleep(0.3)

        # Establish connection
        for _ in range(5):
            pub.publish_pose()
            spin_for(listener, duration=0.1)

        assert listener.is_connected(), "Expected CONNECTED before disconnect"

        # Drop signal
        time.sleep(1.5)
        assert not listener.is_connected(), "Expected NO SIGNAL after dropout"

        # Restore signal
        for _ in range(5):
            pub.publish_pose()
            spin_for(listener, duration=0.1)

        assert listener.is_connected(), \
            "Expected CONNECTED to be restored after resuming publish"

        pub.destroy_node()
        listener.destroy_node()