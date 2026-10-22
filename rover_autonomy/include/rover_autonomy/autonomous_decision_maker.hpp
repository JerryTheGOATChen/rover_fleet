#ifndef ROVER_AUTONOMY__AUTONOMOUS_DECISION_MAKER_HPP_
#define ROVER_AUTONOMY__AUTONOMOUS_DECISION_MAKER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rover_autonomy
{

/// The five high-level behaviors the rover can be in. EXPLORE drives to
/// frontiers (the edge of the known map) until none are left, then returns
/// home. AVOID_OBSTACLE always returns to whichever of EXPLORE/RETURN_HOME it
/// interrupted; EMERGENCY_STOP can be entered from anywhere and only leaves
/// via an explicit service call.
enum class State
{
  IDLE,
  EXPLORE,
  AVOID_OBSTACLE,
  RETURN_HOME,
  EMERGENCY_STOP
};

std::string to_string(State state);

/// Supervisory decision maker. Does NOT plan paths or drive wheels itself
/// except during the reactive AVOID_OBSTACLE / EMERGENCY_STOP states - normal
/// driving is delegated to Nav2 via the navigate_to_pose action, which is the
/// "lower-level" navigation node this module assumes is already running.
class AutonomousDecisionMaker : public rclcpp::Node
{
public:
  AutonomousDecisionMaker();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // --- periodic evaluation (the actual state machine) ---
  void evaluate();
  void transition_to(State new_state);
  void tick_explore();
  bool send_best_frontier();
  bool blacklisted(double x, double y) const;
  void tick_return_home();
  void run_avoid_step();

  // --- sensor callbacks ---
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void try_record_home_pose();
  std::optional<std::pair<double, double>> robot_position();

  // --- Nav2 action client ---
  void send_nav_goal(const geometry_msgs::msg::PoseStamped & pose);
  void cancel_active_goal();
  void nav_goal_response_callback(uint64_t goal_seq, const GoalHandleNav::SharedPtr & goal_handle);
  void nav_result_callback(uint64_t goal_seq, const GoalHandleNav::WrappedResult & result);

  // --- output ---
  void publish_cmd_vel(double linear_x, double angular_z);
  void publish_state();

  // --- mission control services (std_srvs/Trigger: no request fields) ---
  void handle_start_explore(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_return_home(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_clear_emergency(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // --- parameters ---
  double evaluation_rate_hz_{10.0};
  std::string map_frame_{"map"};
  std::string base_frame_{"body_link"};
  double obstacle_distance_{0.6};
  double frontal_half_rad_{0.0};
  double avoid_backup_speed_{-0.15};
  double avoid_turn_speed_{0.6};
  double avoid_min_duration_{2.0};
  double explore_goal_timeout_{60.0};
  double frontier_min_size_{0.5};
  double frontier_distance_weight_{1.0};
  double frontier_size_weight_{1.0};
  double frontier_blacklist_radius_{0.75};
  double frontier_min_goal_distance_{1.0};
  double frontier_min_unknown_area_{4.0};
  double frontier_min_clearance_{0.6};
  double explore_stall_timeout_{60.0};
  double explore_min_progress_{1.0};

  // --- state ---
  State current_state_{State::IDLE};
  State pre_avoid_state_{State::IDLE};
  rclcpp::Time avoid_state_entered_time_;
  bool emergency_stop_active_{false};

  bool obstacle_detected_{false};
  double avoid_turn_direction_{1.0};  // +1 = turn left (ccw), -1 = turn right

  bool home_recorded_{false};
  geometry_msgs::msg::PoseStamped home_pose_;

  // Bumped on every send/cancel. Nav2 callbacks carry the seq of the goal
  // they belong to, so a late result from a cancelled goal can't be mistaken
  // for the result of whatever goal is current now.
  uint64_t goal_seq_{0};
  bool goal_active_{false};
  bool goal_finished_{false};
  bool goal_succeeded_{false};
  GoalHandleNav::SharedPtr current_goal_handle_;

  // --- exploration ---
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  bool has_target_{false};
  double target_x_{0.0};
  double target_y_{0.0};
  rclcpp::Time target_started_time_;  // when this frontier was first chosen
  rclcpp::Time last_frontier_check_;
  std::vector<std::pair<double, double>> blacklist_;  // frontiers Nav2 couldn't reach
  double progress_area_{0.0};   // known-free m^2 at the last real map growth
  rclcpp::Time progress_time_;  // ...and when that was

  // --- ROS interfaces ---
  rclcpp::TimerBase::SharedPtr eval_timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_explore_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_emergency_srv_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__AUTONOMOUS_DECISION_MAKER_HPP_
