import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class GapFollower(Node):    
    # Constants for gap following behavior
    BUBBLE_RADIUS = 50  # Reduce the radius to allow better detection of gaps
    PREPROCESS_CONV_SIZE = 3 # Size for convolution filter during preprocessing
    BEST_POINT_CONV_SIZE = 80  # Size for averaging in the best point detection
    MAX_LIDAR_DIST = 3.0  # Maximum distance to consider from the LiDAR
    STRAIGHTS_SPEED = 2  # Speed while driving straight
    CORNERS_SPEED = 2  # Speed while turning
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # Steering angle threshold for straights

    def __init__(self):
        super().__init__('gap_follower')

        # QoS settings for reliable message delivery
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        # Initialize subscribers and publishers
        self.subscription_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )
        self.publisher_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', qos_profile
        )

        # Set a publishing rate (50Hz)
        self.timer = self.create_timer(0.02, self.publish_drive)

        # Previous values for smoothing
        self.prev_steering_angle = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.radians_per_elem = None  # Placeholder for angle calculations

    def lidar_callback(self, msg):
        """ Callback function for processing LiDAR data """
        ranges = np.array(msg.ranges)
        self.speed, self.steering_angle = self.process_lidar(ranges)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        proc_ranges = np.array(ranges[135:-135])  # Remove rear data
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)  # Limit range values
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Find the start and end indices of the maximum gap """
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)  # Get contiguous free-space slices

        if not slices:  # If no gap is found, return default
            return 0, len(free_space_ranges) - 1

        max_len = 0
        chosen_slice = slices[0]

        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl  # Store the widest gap

        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """ Return index of the best point in ranges within the maximum gap """
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Calculate the steering angle for a given LiDAR range index """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        return lidar_angle / 2  # Adjust steering input

    def process_lidar(self, ranges):
        """ Process each LiDAR scan using the Follow Gap algorithm """
        proc_ranges = self.preprocess_lidar(ranges)

        closest = proc_ranges.argmin()  # Find closest obstacle
        min_index = max(0, closest - self.BUBBLE_RADIUS)
        max_index = min(len(proc_ranges), closest + self.BUBBLE_RADIUS)
        proc_ranges[min_index:max_index] = 0  # Remove bubble area

        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best = self.find_best_point(gap_start, gap_end, proc_ranges)
        steering_angle = self.get_angle(best, len(proc_ranges))

        # Reduce left turn intensity
        #if best < len(proc_ranges) / 2:
        #    steering_angle *= 0.5

        # Apply low-pass filtering for smoother steering
        smoothed_angle = 0.8 * self.prev_steering_angle + 0.2 * steering_angle
        self.prev_steering_angle = smoothed_angle

        # Set speed based on turning
        speed = self.CORNERS_SPEED if abs(smoothed_angle) > self.STRAIGHTS_STEERING_ANGLE else self.STRAIGHTS_SPEED

        self.get_logger().info(f'Steering angle: {smoothed_angle * (180 / np.pi):.2f} degrees')

        return speed, smoothed_angle

    def publish_drive(self):
        """ Publish the latest drive command at a steady rate. """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        self.publisher_drive.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
