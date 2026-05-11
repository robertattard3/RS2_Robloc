import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import time


class EStopListenerNode(Node):
    """Helper node that listens for e-stop messages."""

    def __init__(self):
        super().__init__('estop_listener')
        self.received_msgs = []
        self.subscription = self.create_subscription(
            Bool,
            '/emergency_stop',
            self._callback,
            10
        )

    def _callback(self, msg):
        self.received_msgs.append(msg.data)


class EStopPublisherNode(Node):
    """Helper node that publishes e-stop messages."""

    def __init__(self):
        super().__init__('estop_publisher')
        self.publisher = self.create_publisher(Bool, '/emergency_stop', 10)

    def publish_estop(self, value: bool):
        msg = Bool()
        msg.data = value
        self.publisher.publish(msg)


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(node, duration=1.0):
    """Spin a node for a given duration in a background thread."""
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


class TestEStop:

    def test_estop_publishes_true(self, ros_context):
        """Verify that publishing True on /emergency_stop is received correctly."""
        publisher_node = EStopPublisherNode()
        listener_node = EStopListenerNode()

        # Allow time for discovery
        time.sleep(0.3)

        # Publish e-stop
        publisher_node.publish_estop(True)

        # Spin listener to receive
        spin_for(listener_node, duration=1.0)

        assert len(listener_node.received_msgs) > 0, \
            "No messages received on /emergency_stop"
        assert listener_node.received_msgs[-1] is True, \
            f"Expected True, got {listener_node.received_msgs[-1]}"

        publisher_node.destroy_node()
        listener_node.destroy_node()

    def test_estop_publishes_false(self, ros_context):
        """Verify that a non-estop (False) message is received correctly."""
        publisher_node = EStopPublisherNode()
        listener_node = EStopListenerNode()

        time.sleep(0.3)

        publisher_node.publish_estop(False)
        spin_for(listener_node, duration=1.0)

        assert len(listener_node.received_msgs) > 0, \
            "No messages received on /emergency_stop"
        assert listener_node.received_msgs[-1] is False, \
            f"Expected False, got {listener_node.received_msgs[-1]}"

        publisher_node.destroy_node()
        listener_node.destroy_node()

    def test_estop_multiple_publishes(self, ros_context):
        """Verify that multiple e-stop messages are all received."""
        publisher_node = EStopPublisherNode()
        listener_node = EStopListenerNode()

        time.sleep(0.3)

        for _ in range(5):
            publisher_node.publish_estop(True)
            time.sleep(0.05)

        spin_for(listener_node, duration=1.5)

        assert len(listener_node.received_msgs) >= 5, \
            f"Expected at least 5 messages, got {len(listener_node.received_msgs)}"
        assert all(m is True for m in listener_node.received_msgs), \
            "Not all received messages were True"

        publisher_node.destroy_node()
        listener_node.destroy_node()