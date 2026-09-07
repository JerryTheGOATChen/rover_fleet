// Relays/blocks plain Twist on cmd_vel based on /scan. Fits into the
// architecture either standalone (safety:=true in rover_sim.launch.py) or as
// a layer between manual teleop and the bridge.

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <limits>
#include <string>

class LidarSafetyNode : public rclcpp::Node
{
public:
  LidarSafetyNode() : Node("lidar_safety_node")
  {
    safety_distance_ = declare_parameter<double>("safety_distance", 0.45);
    frontal_angle_deg_ = declare_parameter<double>("frontal_angle_deg", 35.0);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    input_cmd_topic_ = declare_parameter<std::string>("input_cmd_topic", "/cmd_vel");
    output_cmd_topic_ = declare_parameter<std::string>("output_cmd_topic", "/cmd_vel_safe");

    frontal_half_rad_ = frontal_angle_deg_ * M_PI / 180.0;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarSafetyNode::scan_callback, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      std::bind(&LidarSafetyNode::cmd_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "Safety relay: %s -> %s, scan=%s, stop distance=%.2f m, frontal half-angle=%.1f deg",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str(), scan_topic_.c_str(),
      safety_distance_, frontal_angle_deg_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double closest = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (std::abs(angle) > frontal_half_rad_) {
        continue;
      }

      const float range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      closest = std::min(closest, static_cast<double>(range));
    }

    const bool obstacle_now = closest < safety_distance_;
    if (obstacle_now != obstacle_detected_) {
      obstacle_detected_ = obstacle_now;
      if (obstacle_detected_) {
        RCLCPP_WARN(
          get_logger(), "Obstacle at %.2f m inside safety threshold %.2f m",
          closest, safety_distance_);
        publish_stop();
      } else {
        RCLCPP_INFO(get_logger(), "Frontal sector clear");
      }
    } else if (obstacle_detected_) {
      publish_stop();
    }
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (obstacle_detected_) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Safety override active; blocking commanded velocity");
      return;
    }

    cmd_pub_->publish(*msg);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  double safety_distance_;
  double frontal_angle_deg_;
  double frontal_half_rad_;
  bool obstacle_detected_{false};
  std::string scan_topic_;
  std::string input_cmd_topic_;
  std::string output_cmd_topic_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
