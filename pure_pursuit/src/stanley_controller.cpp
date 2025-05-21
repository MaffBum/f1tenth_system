#include "stanley_controller.hpp"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>
#include <sstream>

StanleyController::StanleyController() : Node("stanley_controller_node") {
    declare_parameter("waypoints_path", "/home/f1jetson/f1tenth_ws/src/f1tenth_system/pure_pursuit/racelines/31march.csv");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("car_refFrame", "laser");
    declare_parameter("drive_topic", "/drive");
    declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
    declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
    declare_parameter("global_refFrame", "map");
    declare_parameter("stanley_gain", 1.0);
    declare_parameter("steering_limit", 25.0);
    declare_parameter("velocity_percentage", 0.6);

    waypoints_path = get_parameter("waypoints_path").as_string();
    odom_topic = get_parameter("odom_topic").as_string();
    car_refFrame = get_parameter("car_refFrame").as_string();
    drive_topic = get_parameter("drive_topic").as_string();
    rviz_current_waypoint_topic = get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic = get_parameter("rviz_lookahead_waypoint_topic").as_string();
    global_refFrame = get_parameter("global_refFrame").as_string();
    K_p = get_parameter("stanley_gain").as_double();
    steering_limit = get_parameter("steering_limit").as_double();
    velocity_percentage = get_parameter("velocity_percentage").as_double();

    publisher_drive = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_current_point_pub = create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_timer_ = create_wall_timer(20ms, std::bind(&StanleyController::control_loop, this));
    timer_ = create_wall_timer(2000ms, std::bind(&StanleyController::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Stanley controller node has been launched");
    load_waypoints();
}

void StanleyController::load_waypoints() {
    csvFile_waypoints.open(waypoints_path);
    std::string line;
    while (getline(csvFile_waypoints, line)) {
        std::stringstream s(line);
        std::string x, y, v;
        getline(s, x, ',');
        getline(s, y, ',');
        getline(s, v, ',');
        waypoints.X.push_back(std::stod(x));
        waypoints.Y.push_back(std::stod(y));
        waypoints.V.push_back(std::stod(v));
    }
    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();
    RCLCPP_INFO(get_logger(), "Loaded %d waypoints", num_waypoints);
}

void StanleyController::get_waypoint() {
    double shortest = std::numeric_limits<double>::max();
    for (int i = 0; i < num_waypoints; ++i) {
        double dist = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
        if (dist < shortest) {
            shortest = dist;
            waypoints.index = i;
        }
    }
    waypoints.velocity_index = waypoints.index;
}

double StanleyController::stanley_control() {
    double dx = waypoints.X[waypoints.index] - x_car_world;
    double dy = waypoints.Y[waypoints.index] - y_car_world;
    double path_yaw = std::atan2(dy, dx);
    double heading_error = path_yaw - yaw_car_world;

    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    double cross_track = -std::sin(yaw_car_world) * dx + std::cos(yaw_car_world) * dy;
    return heading_error + std::atan2(K_p * cross_track, curr_velocity + 1e-3);
}

double StanleyController::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double StanleyController::get_velocity(double steering_angle) {
    if (waypoints.V[waypoints.velocity_index] > 0.1)
        return waypoints.V[waypoints.velocity_index] * velocity_percentage;
    return 2.0 * velocity_percentage;
}

void StanleyController::quat_to_rot(double q0, double q1, double q2, double q3) {
    rotation_m <<
        2 * (q0*q0 + q1*q1) - 1, 2 * (q1*q2 - q0*q3),     2 * (q1*q3 + q0*q2),
        2 * (q1*q2 + q0*q3),     2 * (q0*q0 + q2*q2) - 1, 2 * (q2*q3 - q0*q1),
        2 * (q1*q3 - q0*q2),     2 * (q2*q3 + q0*q1),     2 * (q0*q0 + q3*q3) - 1;
}

void StanleyController::transform_and_interp_waypoint() {
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;
    visualize_point(waypoints.lookahead_point_world, vis_lookahead_point_pub, 1.0, 0.0, 0.0);
    visualize_point(waypoints.current_point_world, vis_current_point_pub, 0.0, 0.0, 1.0);
}

void StanleyController::visualize_point(Eigen::Vector3d &point, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    pub->publish(marker);
}

void StanleyController::publish_message(double steering_angle) {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.drive.steering_angle = std::clamp(steering_angle, -to_radians(steering_limit), to_radians(steering_limit));
    curr_velocity = get_velocity(msg.drive.steering_angle);
    msg.drive.speed = curr_velocity;
    publisher_drive->publish(msg);
}

void StanleyController::control_loop() {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(global_refFrame, car_refFrame, tf2::TimePointZero);
        x_car_world = tf.transform.translation.x;
        y_car_world = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(std::ignore, std::ignore, yaw_car_world);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        return;
    }

    get_waypoint();
    transform_and_interp_waypoint();
    double steering_angle = stanley_control();
    publish_message(steering_angle);
}

void StanleyController::timer_callback() {
    K_p = get_parameter("stanley_gain").as_double();
    velocity_percentage = get_parameter("velocity_percentage").as_double();
    steering_limit = get_parameter("steering_limit").as_double();
}

double StanleyController::to_radians(double degrees) { return degrees * M_PI / 180.0; }
double StanleyController::to_degrees(double radians) { return radians * 180.0 / M_PI; }
