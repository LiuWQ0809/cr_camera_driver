#!/usr/bin/env python3
import argparse
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class H265Saver(Node):
    def __init__(self, topic, output_path, max_msgs):
        super().__init__('h265_topic_saver')
        self._topic = topic
        self._output_path = output_path
        self._max_msgs = max_msgs
        self._count = 0
        os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
        self._fh = open(output_path, 'ab')
        self.create_subscription(CompressedImage, topic, self._callback, 10)
        self.get_logger().info(f"Saving H265 stream from {topic} -> {output_path}")

    def _callback(self, msg):
        if msg.format != 'h265':
            self.get_logger().warn(f"Unexpected format: {msg.format}")
        if msg.data:
            self._fh.write(bytes(msg.data))
            self._fh.flush()
        self._count += 1
        if self._max_msgs > 0 and self._count >= self._max_msgs:
            self.get_logger().info(f"Saved {self._count} messages, exiting")
            self._fh.close()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Save CompressedImage(H265) topic to a .h265 file')
    parser.add_argument('--topic', default='/cr/camera/h265/front_right', help='H265 topic name')
    parser.add_argument('--output', default='/tmp/camera_front_right.h265', help='Output .h265 file')
    parser.add_argument('--max-msgs', type=int, default=300, help='Stop after N messages (0 = run forever)')
    args = parser.parse_args()

    rclpy.init()
    node = H265Saver(args.topic, args.output, args.max_msgs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._fh.closed:
            node._fh.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
