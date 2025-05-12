import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

class AMCLLogger(Node):
    def __init__(self):
        super().__init__('amcl_logger')

        self.declare_parameter('csv_filename', 'amcl_output.csv')
        filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        self.csv_path = os.path.join(
            os.path.dirname(__file__), '..', 'csv', filename)

        self.get_logger().info(f'Logging AMCL data to {self.csv_path}')

        self.scan_timestamp = None

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'computation_time'])

    def scan_callback(self, msg):
        self.scan_timestamp = self.get_clock().now()

    def pose_callback(self, msg):
        if self.scan_timestamp is None:
            return
        now = self.get_clock().now()
        comp_time = (now.nanoseconds - self.scan_timestamp.nanoseconds) / 1e6  # ms

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        timestamp = now.seconds_nanoseconds()[0] + (now.seconds_nanoseconds()[1] * 1e-9)

        self.csv_writer.writerow([timestamp, x, y, comp_time])
        self.get_logger().info(f'[AMCL] x={x:.2f}, y={y:.2f}, Δt={comp_time:.2f}ms')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMCLLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
