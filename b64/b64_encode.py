import base64

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


class B64EncodeNode(Node):
    """Node that subscribes to any topic and publishes base64 encoded data."""

    def __init__(self):
        super().__init__('b64_encode')

        self.declare_parameter('input_topic', '/input')
        self.declare_parameter('input_type', 'std_msgs/msg/String')

        input_topic = self.get_parameter('input_topic').value
        input_type = self.get_parameter('input_type').value

        msg_class = get_message(input_type)

        self.subscription = self.create_subscription(
            msg_class,
            input_topic,
            self._callback,
            10
        )

        self.publisher = self.create_publisher(String, '~/output', 10)

        self.get_logger().info(
            f'Subscribing to {input_topic} ({input_type}), '
            f'publishing to ~/output'
        )

    def _callback(self, msg):
        serialized_msg = serialize_message(msg)
        encoded = base64.b64encode(serialized_msg).decode('ascii')
        out_msg = String()
        out_msg.data = encoded
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = B64EncodeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
