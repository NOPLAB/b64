import base64

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class B64EncodeNode(Node):
    """Node that subscribes to any topic and publishes base64 encoded data."""

    def __init__(self):
        super().__init__('b64_encode')

        self.declare_parameter('input_topic', '/input')
        self.declare_parameter('input_type', 'std_msgs/msg/String')

        input_topic = self.get_parameter('input_topic').value
        input_type = self.get_parameter('input_type').value

        qos = QoSProfile(depth=10)

        self.subscription = self.create_generic_subscription(
            input_type,
            input_topic,
            self._callback,
            qos
        )

        self.publisher = self.create_publisher(String, '~/output', 10)

        self.get_logger().info(
            f'Subscribing to {input_topic} ({input_type}), '
            f'publishing to ~/output'
        )

    def _callback(self, serialized_msg: bytes):
        encoded = base64.b64encode(serialized_msg).decode('ascii')
        out_msg = String()
        out_msg.data = encoded
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = B64EncodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
