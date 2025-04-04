import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class DriveBridge(Node):
    def __init__(self):
        super().__init__('drive_bridge')
        
        self.sub_drive = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)

    def drive_callback(self, msg):
        throttle_msg = Float32()
        steering_msg = Float32()

        throttle_msg.data = msg.drive.speed
        steering_msg.data = msg.drive.steering_angle

        self.pub_throttle.publish(throttle_msg)
        self.pub_steering.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()