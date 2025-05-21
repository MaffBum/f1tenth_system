#ifndef STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <fstream>
#include <string>

class StanleyController : public rclcpp::Node {
public:
    StanleyController();

private:
    struct Waypoints {
        std::vector<double> X, Y, V;
        int index = 0;
        int velocity_index = 0;
        Eigen::Vector3d lookahead_point_world;
        Eigen::Vector3d lookahead_point_car;
        Eigen::Vector3d current_point_world;
    };

    void load_waypoints();
    void get_waypoint();
    void transform_and_interp_waypoint();
    void visualize_point(Eigen::Vector3d &point, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, float r, float g, float b);
    void quat_to_rot(double q0, double q1, double q2, double q3);
    void control_loop();
    double get_velocity(double steering_angle);
    double stanley_control();
    double to_radians(double degrees);
    double to_degrees(double radians);
    double p2pdist(double &x1, double &x2, double &y1, double &y2);
    void publish_message(double steering_angle);
    void timer_callback();

    Waypoints waypoints;
    int num_waypoints;

    std::string waypoints_path, odom_topic, car_refFrame, drive_topic;
    std::string rviz_current_waypoint_topic, rviz_lookahead_waypoint_topic, global_refFrame;

    rclcpp::TimerBase::SharedPtr timer_, control_timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    double curr_velocity, K_p, steering_limit, velocity_percentage;
    double x_car_world, y_car_world, yaw_car_world;
    Eigen::Matrix3d rotation_m;
    std::ifstream csvFile_waypoints;
};

#endif  // STANLEY_CONTROLLER_HPP
