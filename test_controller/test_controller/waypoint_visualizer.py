import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import csv
import os

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer_node')
        
        # Declare parameters
        self.declare_parameter('waypoints_path', '/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv')
        self.declare_parameter('rviz_waypoints_topic', '/waypoints')

        # Get parameters
        self.waypoints_path = self.get_parameter('waypoints_path').get_parameter_value().string_value
        self.rviz_waypoints_topic = self.get_parameter('rviz_waypoints_topic').get_parameter_value().string_value

        # Publisher for visualization
        self.vis_path_pub = self.create_publisher(MarkerArray, self.rviz_waypoints_topic, 1000)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('This node has been launched')
        self.waypoints = {'X': [], 'Y': []}
        self.load_waypoints()

    def load_waypoints(self):
        if not os.path.exists(self.waypoints_path):
            self.get_logger().error(f"File not found: {self.waypoints_path}")
            return

        with open(self.waypoints_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 2:  # Ensure there are enough columns
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        self.waypoints['X'].append(x)
                        self.waypoints['Y'].append(y)
                        self.get_logger().info(f'{x}... X point')
                        self.get_logger().info(f'{y}... Y point')
                    except ValueError:
                        self.get_logger().warn("Invalid data in CSV file.")

    def visualize_points(self):
        marker_array = MarkerArray()
        for i in range(len(self.waypoints['X'])):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.pose.position.x = self.waypoints['X'][i]
            marker.pose.position.y = self.waypoints['Y'][i]
            marker.id = i
            marker_array.markers.append(marker)

        self.vis_path_pub.publish(marker_array)

    def timer_callback(self):
        self.visualize_points()

def main(args=None):
    rclpy.init(args=args)
    waypoint_visualizer = WaypointVisualizer()
    rclpy.spin(waypoint_visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
