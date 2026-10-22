// The last stop for every velocity command: relays twist_mux's output
// (/cmd_vel_mux) to the drive (/cmd_vel_safe), clamping forward motion to
// zero while /scan sees something inside safety_distance in the frontal
// sector. Reversing and turning always pass through, so you can still drive
// away from whatever tripped it. Set safety_distance to 0 to disable.
//
// Topic names are relative, so in a namespace (fleet: /robot1/...) the same
// node gates that robot's own scan and commands.

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rover_autonomy/scan_utils.hpp"

#include <algorithm>
#include <cmath>

class LidarSafetyNode : public rclcpp::Node
{
public:
  LidarSafetyNode() : Node("lidar_safety_node")
  {
    safety_distance_ = declare_parameter<double>("safety_distance", 0.45);
    const double frontal_angle_deg = declare_parameter<double>("frontal_angle_deg", 35.0);

    frontal_half_rad_ = frontal_angle_deg * M_PI / 180.0;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&LidarSafetyNode::scan_callback, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_mux", 10,
      std::bind(&LidarSafetyNode::cmd_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);

    RCLCPP_INFO(
      get_logger(), "Safety gate cmd_vel_mux -> cmd_vel_safe: stop %.2f m, half-angle %.1f deg",
      safety_distance_, frontal_angle_deg);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto sector = rover_autonomy::analyze_front_sector(*msg, frontal_half_rad_);
    const bool obstacle_now = sector.closest < safety_distance_;
    if (obstacle_now == obstacle_detected_) {
      return;
    }

    obstacle_detected_ = obstacle_now;
    if (obstacle_detected_) {
      RCLCPP_WARN(
        get_logger(), "Obstacle at %.2f m inside safety threshold %.2f m",
        sector.closest, safety_distance_);
      // The sim's DiffDrive plugin holds the last command it received, so an
      // explicit stop is needed here in case the input source has gone quiet
      // mid-drive (e.g. teleop key released after a forward command).
      if (last_forwarded_.linear.x > 0.0) {
        auto stop = last_forwarded_;
        stop.linear.x = 0.0;
        publish(stop);
      }
    } else {
      RCLCPP_INFO(get_logger(), "Frontal sector clear");
    }
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto cmd = *msg;
    if (obstacle_detected_ && cmd.linear.x > 0.0) {
      cmd.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Safety override active; blocking forward motion (reverse/turn still allowed)");
    }
    publish(cmd);
  }

  void publish(const geometry_msgs::msg::Twist & cmd)
  {
    last_forwarded_ = cmd;
    cmd_pub_->publish(cmd);
  }

  double safety_distance_;
  double frontal_half_rad_;
  bool obstacle_detected_{false};
  geometry_msgs::msg::Twist last_forwarded_;

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
