#include <chrono>
#include <memory>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class AmclTimingNode : public rclcpp::Node
{
public:
  AmclTimingNode()
  : Node("amcl_timing_node")
  {
    csv_file_.open("/home/f1jetson/f1tenth_ws/src/f1tenth_system/amcl_timing/csvs/amcl_latency_log.csv", std::ios::out | std::ios::app);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
    }

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_received_time_ = this->now();
      });

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        rclcpp::Time pose_received_time = this->now();
        rclcpp::Duration latency = pose_received_time - last_scan_received_time_;
        double latency_ms = latency.seconds() * 1000.0;

        if (latency.nanoseconds() >= 0) {
          RCLCPP_INFO(this->get_logger(), "AMCL latency: %.3f ms", latency_ms);
          if (csv_file_.is_open()) {
            csv_file_ << std::fixed << std::setprecision(3) << latency_ms << "\n";
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Negative latency: %.3f ms", latency_ms);
        }
      });

    RCLCPP_INFO(this->get_logger(), "AMCL timing node started");
  }

  ~AmclTimingNode() override
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  rclcpp::Time last_scan_received_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclTimingNode>());
  rclcpp::shutdown();
  return 0;
}
