// Twist -> TwistStamped converter for the real robot's diff_drive_controller
// (use_stamped_vel: true). See src/rover_control_node.cpp.

#ifndef ROVER_CONTROL_NODE_HPP_
#define ROVER_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace keyboard_control
{
/*
Subscribes to keyboard twist inputs and publishes cmd_vel
*/

class RoverControlNode : public rclcpp::Node
{
public:
    RoverControlNode();
private:
    //Publishing cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    void twistCallback(const geometry_msgs::msg::Twist & msg);
};



}

#endif //ROVER_CONTROL_NODE_HPP_
