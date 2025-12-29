import base64

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class B64DecodeNode(Node):
    """Node that subscribes to base64 string and publishes decoded raw bytes."""

    def __init__(self):
        super().__init__('b64_decode')

        self.declare_parameter('output_topic', '/output')
        self.declare_parameter('output_type', 'std_msgs/msg/String')

        output_topic = self.get_parameter('output_topic').value
        output_type = self.get_parameter('output_type').value

        self.subscription = self.create_subscription(
            String,
            '~/input',
            self._callback,
            10
        )

        qos = QoSProfile(depth=10)

        self.publisher = self.create_generic_publisher(
            output_type,
            output_topic,
            qos
        )

        self.get_logger().info(
            f'Subscribing to ~/input, '
            f'publishing to {output_topic} ({output_type})'
        )

    def _callback(self, msg: String):
        try:
            decoded = base64.b64decode(msg.data)
            self.publisher.publish(decoded)
        except Exception as e:
            self.get_logger().error(f'Failed to decode base64: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = B64DecodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
