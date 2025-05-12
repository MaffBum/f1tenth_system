#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import csv
import atexit

class TFLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')
        self.sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        # Open CSV without writing headers
        self.csv_file = open('tf_listener/data/tf_data.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        atexit.register(self.csv_file.close)

        self.get_logger().info('tf_logger node started, logging timestamp,x,y to tf_data.csv.')

    def tf_callback(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id != 'ego_racecar/base_link':
                continue

            # build timestamp (seconds + nanoseconds)
            ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            x  = t.transform.translation.x
            y  = t.transform.translation.y

            # write timestamp, x, y (no header)
            self.writer.writerow([f"{ts:.9f}", x, y])

            # optional console log
            self.get_logger().info(f"[{ts:.3f}] x={x:.3f}, y={y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = TFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down tf_logger...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

