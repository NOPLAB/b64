import base64

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


class B64DecodeNode(Node):
    """Node that subscribes to base64 string and publishes decoded raw bytes."""

    def __init__(self):
        super().__init__('b64_decode')

        self.declare_parameter('output_topic', '/output')
        self.declare_parameter('output_type', 'std_msgs/msg/String')

        output_topic = self.get_parameter('output_topic').value
        output_type = self.get_parameter('output_type').value

        self.msg_class = get_message(output_type)

        self.subscription = self.create_subscription(
            String,
            '~/input',
            self._callback,
            10
        )

        self.publisher = self.create_publisher(
            self.msg_class,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Subscribing to ~/input, '
            f'publishing to {output_topic} ({output_type})'
        )

    def _callback(self, msg: String):
        try:
            decoded = base64.b64decode(msg.data)
            out_msg = deserialize_message(decoded, self.msg_class)
            self.publisher.publish(out_msg)
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
