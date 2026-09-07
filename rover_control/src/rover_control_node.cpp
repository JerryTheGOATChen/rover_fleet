// Twist -> TwistStamped converter. Built and used again as of the real-
// hardware bring-up: controllers.yaml's diff_drive_controller runs with
// use_stamped_vel: true, so it expects TwistStamped on its cmd_vel topic
// (unlike the sim, where the native gz-sim DiffDrive plugin takes plain
// Twist). Wired in by rover_hardware.launch.py.

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToStampedNode : public rclcpp::Node
{
public:
  TwistToStampedNode() : Node("twist_to_stamped_cmd_vel")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "body_link");

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10,
      std::bind(&TwistToStampedNode::twist_callback, this, std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);

    RCLCPP_INFO(
      get_logger(), "Converting Twist %s -> TwistStamped %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = frame_id_;
    stamped.twist = *msg;
    pub_->publish(stamped);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToStampedNode>());
  rclcpp::shutdown();
  return 0;
}
