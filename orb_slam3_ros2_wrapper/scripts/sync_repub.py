#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

class StereoSyncRepublisher(Node):
    def __init__(self):
        super().__init__('stereo_sync_republisher')

        # Params
        self.declare_parameter('left_topic',  '/stereo/left/image')
        self.declare_parameter('right_topic', '/stereo/right/image')
        self.declare_parameter('out_left_topic',  '/x500_depth_1/image_left')
        self.declare_parameter('out_right_topic', '/x500_depth_1/image_right')

        # Para tu jitter (max 0.26s), necesitas cola grande.
        # slop: tolerancia de desfase entre stamps para emparejar.
        self.declare_parameter('queue_size', 50)
        self.declare_parameter('slop', 0.05)  # 50 ms para empezar; ajusta si hace falta
        self.declare_parameter('force_same_stamp', True)

        left_topic  = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        out_left    = self.get_parameter('out_left_topic').value
        out_right   = self.get_parameter('out_right_topic').value

        self.force_same_stamp = bool(self.get_parameter('force_same_stamp').value)

        # Subs (message_filters)
        self.left_sub  = Subscriber(self, Image, left_topic)
        self.right_sub = Subscriber(self, Image, right_topic)

        queue_size = int(self.get_parameter('queue_size').value)
        slop = float(self.get_parameter('slop').value)

        self.sync = ApproximateTimeSynchronizer(
            fs=[self.left_sub, self.right_sub],
            queue_size=queue_size,
            slop=slop
        )
        self.sync.registerCallback(self.cb)

        # Pubs
        self.pub_left  = self.create_publisher(Image, out_left, 10)
        self.pub_right = self.create_publisher(Image, out_right, 10)

        self.get_logger().info(f"Syncing:\n  L: {left_topic}\n  R: {right_topic}\nPublishing:\n  L: {out_left}\n  R: {out_right}")

    def cb(self, left: Image, right: Image):
        if self.force_same_stamp:
            # Usa el menor (más conservador) o podrías promediar
            stamp = left.header.stamp if (left.header.stamp.sec, left.header.stamp.nanosec) <= (right.header.stamp.sec, right.header.stamp.nanosec) else right.header.stamp
            left.header.stamp = stamp
            right.header.stamp = stamp

        self.pub_left.publish(left)
        self.pub_right.publish(right)

def main():
    rclpy.init()
    node = StereoSyncRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
