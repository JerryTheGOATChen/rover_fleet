#include "rover_control/autonomous/autonomous_decision_maker.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "tf2/exceptions.h"

namespace autonomous
{

std::string to_string(State state)
{
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::EXPLORE: return "EXPLORE";
    case State::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
    case State::RETURN_HOME: return "RETURN_HOME";
    case State::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

AutonomousDecisionMaker::AutonomousDecisionMaker()
: Node("autonomous_decision_maker")
{
  evaluation_rate_hz_ = declare_parameter<double>("evaluation_rate_hz", 10.0);
  const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto emergency_stop_topic =
    declare_parameter<std::string>("emergency_stop_topic", "/autonomy/emergency_stop");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/autonomy/state");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "body_link");

  obstacle_distance_ = declare_parameter<double>("obstacle_distance", 0.6);
  const double frontal_angle_deg = declare_parameter<double>("obstacle_frontal_angle_deg", 40.0);
  frontal_half_rad_ = frontal_angle_deg * M_PI / 180.0;

  avoid_backup_speed_ = declare_parameter<double>("avoid_backup_speed", -0.15);
  avoid_turn_speed_ = declare_parameter<double>("avoid_turn_speed", 0.6);
  avoid_min_duration_ = declare_parameter<double>("avoid_min_duration", 2.0);

  explore_radius_ = declare_parameter<double>("explore_radius", 2.5);
  explore_goal_timeout_ = declare_parameter<double>("explore_goal_timeout", 60.0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&AutonomousDecisionMaker::scan_callback, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic, rclcpp::QoS(10),
    std::bind(&AutonomousDecisionMaker::emergency_stop_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));
  state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, rclcpp::QoS(10));

  start_explore_srv_ = create_service<std_srvs::srv::Trigger>(
    "/autonomy/start_explore",
    std::bind(
      &AutonomousDecisionMaker::handle_start_explore, this,
      std::placeholders::_1, std::placeholders::_2));
  return_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/autonomy/return_home",
    std::bind(
      &AutonomousDecisionMaker::handle_return_home, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "/autonomy/stop",
    std::bind(
      &AutonomousDecisionMaker::handle_stop, this,
      std::placeholders::_1, std::placeholders::_2));
  clear_emergency_srv_ = create_service<std_srvs::srv::Trigger>(
    "/autonomy/clear_emergency",
    std::bind(
      &AutonomousDecisionMaker::handle_clear_emergency, this,
      std::placeholders::_1, std::placeholders::_2));

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  const auto period = std::chrono::duration<double>(1.0 / evaluation_rate_hz_);
  eval_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&AutonomousDecisionMaker::evaluate, this));

  RCLCPP_INFO(get_logger(), "autonomous_decision_maker up, starting in IDLE");
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

void AutonomousDecisionMaker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double closest = std::numeric_limits<double>::infinity();
  double left_sum = 0.0;
  double right_sum = 0.0;
  int left_count = 0;
  int right_count = 0;

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
    if (angle >= 0.0) {
      left_sum += range;
      ++left_count;
    } else {
      right_sum += range;
      ++right_count;
    }
  }

  obstacle_detected_ = closest < obstacle_distance_;

  // Turn toward whichever side has more room, so the avoidance maneuver is a
  // little smarter than "always spin the same way."
  const double left_avg = left_count > 0 ? left_sum / left_count : 0.0;
  const double right_avg = right_count > 0 ? right_sum / right_count : 0.0;
  avoid_turn_direction_ = (left_avg >= right_avg) ? 1.0 : -1.0;
}

void AutonomousDecisionMaker::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // One-directional on purpose: this can only raise the flag. Clearing it
  // requires the explicit /autonomy/clear_emergency service, so a momentary
  // blip on this topic can't silently self-clear the stop.
  if (msg->data) {
    emergency_stop_active_ = true;
  }
}

void AutonomousDecisionMaker::try_record_home_pose()
{
  if (home_recorded_) {
    return;
  }

  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    home_pose_.header.frame_id = map_frame_;
    home_pose_.header.stamp = now();
    home_pose_.pose.position.x = tf.transform.translation.x;
    home_pose_.pose.position.y = tf.transform.translation.y;
    home_pose_.pose.position.z = 0.0;
    home_pose_.pose.orientation = tf.transform.rotation;
    home_recorded_ = true;
    RCLCPP_INFO(
      get_logger(), "Recorded home pose at (%.2f, %.2f)",
      home_pose_.pose.position.x, home_pose_.pose.position.y);
  } catch (const tf2::TransformException &) {
    // map -> base_frame isn't published yet (slam_toolbox needs a scan or two
    // before its first transform) - completely normal right after startup,
    // we just try again next tick.
  }
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

void AutonomousDecisionMaker::evaluate()
{
  try_record_home_pose();

  if (emergency_stop_active_ && current_state_ != State::EMERGENCY_STOP) {
    transition_to(State::EMERGENCY_STOP);
  }

  switch (current_state_) {
    case State::IDLE:
      break;

    case State::EXPLORE:
      if (obstacle_detected_) {
        transition_to(State::AVOID_OBSTACLE);
      } else {
        tick_explore();
      }
      break;

    case State::AVOID_OBSTACLE:
      run_avoid_step();
      if (!obstacle_detected_ &&
        (now() - avoid_state_entered_time_).seconds() > avoid_min_duration_)
      {
        transition_to(pre_avoid_state_);
      }
      break;

    case State::RETURN_HOME:
      if (obstacle_detected_) {
        transition_to(State::AVOID_OBSTACLE);
      } else {
        tick_return_home();
      }
      break;

    case State::EMERGENCY_STOP:
      publish_cmd_vel(0.0, 0.0);
      break;
  }

  publish_state();
}

void AutonomousDecisionMaker::transition_to(State new_state)
{
  if (new_state == current_state_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "[autonomy] %s -> %s",
    to_string(current_state_).c_str(), to_string(new_state).c_str());

  // Every state change means "the previous plan no longer applies" - drop
  // whatever Nav2 goal was in flight. Nav2 goals are cheap to re-issue.
  cancel_active_goal();

  if (new_state == State::AVOID_OBSTACLE) {
    pre_avoid_state_ = current_state_;
    avoid_state_entered_time_ = now();
  }

  if (new_state == State::EXPLORE || new_state == State::RETURN_HOME) {
    goal_finished_ = false;
    goal_succeeded_ = false;
  }

  if (new_state == State::IDLE || new_state == State::EMERGENCY_STOP) {
    publish_cmd_vel(0.0, 0.0);
  }

  current_state_ = new_state;
}

void AutonomousDecisionMaker::tick_explore()
{
  if (goal_active_) {
    if ((now() - goal_sent_time_).seconds() > explore_goal_timeout_) {
      RCLCPP_WARN(
        get_logger(), "Explore goal taking longer than %.0fs, picking a new one",
        explore_goal_timeout_);
      cancel_active_goal();
    } else {
      return;
    }
  }

  // Placeholder exploration policy: wander to a random point within
  // explore_radius_ of home and let Nav2 plan/drive there. This does NOT
  // look at the map for unexplored space - swap it for a frontier-based
  // explorer (e.g. a package like m-explore-ros2 querying /map) if you want
  // the robot to actually prioritize unmapped areas instead of wandering.
  send_nav_goal(random_pose_near_home());
}

void AutonomousDecisionMaker::tick_return_home()
{
  if (goal_active_) {
    return;
  }

  if (!goal_finished_) {
    send_nav_goal(home_pose_);
    return;
  }

  if (goal_succeeded_) {
    RCLCPP_INFO(get_logger(), "Arrived home");
    transition_to(State::IDLE);
  } else {
    RCLCPP_WARN(get_logger(), "Return-home navigation failed, retrying");
    send_nav_goal(home_pose_);
  }
}

void AutonomousDecisionMaker::run_avoid_step()
{
  if (obstacle_detected_) {
    publish_cmd_vel(avoid_backup_speed_, avoid_turn_speed_ * avoid_turn_direction_);
  } else {
    publish_cmd_vel(0.0, 0.0);
  }
}

// ---------------------------------------------------------------------------
// Nav2 action client
// ---------------------------------------------------------------------------

void AutonomousDecisionMaker::send_nav_goal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "navigate_to_pose action server not available - is Nav2 running?");
    return;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = std::bind(
    &AutonomousDecisionMaker::nav_goal_response_callback, this, std::placeholders::_1);
  options.result_callback = std::bind(
    &AutonomousDecisionMaker::nav_result_callback, this, std::placeholders::_1);

  goal_active_ = true;
  goal_finished_ = false;
  goal_succeeded_ = false;
  goal_sent_time_ = now();

  nav_client_->async_send_goal(goal_msg, options);
}

void AutonomousDecisionMaker::cancel_active_goal()
{
  if (goal_active_ && current_goal_handle_) {
    nav_client_->async_cancel_goal(current_goal_handle_);
  }
  goal_active_ = false;
  current_goal_handle_.reset();
}

geometry_msgs::msg::PoseStamped AutonomousDecisionMaker::random_pose_near_home()
{
  std::uniform_real_distribution<double> radius_dist(0.5, explore_radius_);
  std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
  const double r = radius_dist(rng_);
  const double theta = angle_dist(rng_);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map_frame_;
  pose.header.stamp = now();
  pose.pose.position.x = home_pose_.pose.position.x + r * std::cos(theta);
  pose.pose.position.y = home_pose_.pose.position.y + r * std::sin(theta);
  pose.pose.orientation.w = 1.0;
  return pose;
}

void AutonomousDecisionMaker::nav_goal_response_callback(
  const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Navigation goal was rejected by Nav2");
    goal_active_ = false;
    goal_finished_ = true;
    goal_succeeded_ = false;
    return;
  }
  current_goal_handle_ = goal_handle;
}

void AutonomousDecisionMaker::nav_result_callback(const GoalHandleNav::WrappedResult & result)
{
  goal_active_ = false;
  goal_finished_ = true;
  goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  current_goal_handle_.reset();
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

void AutonomousDecisionMaker::publish_cmd_vel(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_vel_pub_->publish(msg);
}

void AutonomousDecisionMaker::publish_state()
{
  std_msgs::msg::String msg;
  msg.data = to_string(current_state_);
  state_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Mission-control services
// ---------------------------------------------------------------------------

void AutonomousDecisionMaker::handle_start_explore(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ == State::EMERGENCY_STOP) {
    response->success = false;
    response->message = "Emergency stop is active; clear it first.";
    return;
  }
  if (!home_recorded_) {
    response->success = false;
    response->message = "Home pose not recorded yet (waiting on " +
      map_frame_ + " -> " + base_frame_ + " TF).";
    return;
  }
  transition_to(State::EXPLORE);
  response->success = true;
  response->message = "Exploring.";
}

void AutonomousDecisionMaker::handle_return_home(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ == State::EMERGENCY_STOP) {
    response->success = false;
    response->message = "Emergency stop is active; clear it first.";
    return;
  }
  if (!home_recorded_) {
    response->success = false;
    response->message = "No home pose recorded yet.";
    return;
  }
  transition_to(State::RETURN_HOME);
  response->success = true;
  response->message = "Returning home.";
}

void AutonomousDecisionMaker::handle_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ == State::EMERGENCY_STOP) {
    response->success = false;
    response->message = "Already stopped harder than this - use clear_emergency instead.";
    return;
  }
  transition_to(State::IDLE);
  response->success = true;
  response->message = "Stopped, back to idle.";
}

void AutonomousDecisionMaker::handle_clear_emergency(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ != State::EMERGENCY_STOP) {
    response->success = false;
    response->message = "Not currently in emergency stop.";
    return;
  }
  emergency_stop_active_ = false;
  transition_to(State::IDLE);
  response->success = true;
  response->message = "Emergency stop cleared.";
}

}  // namespace autonomous

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomous::AutonomousDecisionMaker>());
  rclcpp::shutdown();
  return 0;
}
