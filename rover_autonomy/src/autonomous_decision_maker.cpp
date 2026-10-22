#include "rover_autonomy/autonomous_decision_maker.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "rover_autonomy/frontier.hpp"
#include "rover_autonomy/scan_utils.hpp"
#include "tf2/exceptions.h"

namespace rover_autonomy
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
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "body_link");

  obstacle_distance_ = declare_parameter<double>("obstacle_distance", 0.6);
  const double frontal_angle_deg = declare_parameter<double>("obstacle_frontal_angle_deg", 40.0);
  frontal_half_rad_ = frontal_angle_deg * M_PI / 180.0;

  avoid_backup_speed_ = declare_parameter<double>("avoid_backup_speed", -0.15);
  avoid_turn_speed_ = declare_parameter<double>("avoid_turn_speed", 0.6);
  avoid_min_duration_ = declare_parameter<double>("avoid_min_duration", 2.0);

  explore_goal_timeout_ = declare_parameter<double>("explore_goal_timeout", 60.0);
  frontier_min_size_ = declare_parameter<double>("frontier_min_size", 0.5);
  frontier_distance_weight_ = declare_parameter<double>("frontier_distance_weight", 1.0);
  frontier_size_weight_ = declare_parameter<double>("frontier_size_weight", 1.0);
  frontier_blacklist_radius_ = declare_parameter<double>("frontier_blacklist_radius", 0.75);
  frontier_min_goal_distance_ = declare_parameter<double>("frontier_min_goal_distance", 1.0);
  frontier_min_unknown_area_ = declare_parameter<double>("frontier_min_unknown_area", 4.0);
  frontier_min_clearance_ = declare_parameter<double>("frontier_min_clearance", 0.6);
  explore_stall_timeout_ = declare_parameter<double>("explore_stall_timeout", 60.0);
  explore_min_progress_ = declare_parameter<double>("explore_min_progress", 1.0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&AutonomousDecisionMaker::scan_callback, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "autonomy/emergency_stop", rclcpp::QoS(10),
    std::bind(&AutonomousDecisionMaker::emergency_stop_callback, this, std::placeholders::_1));

  // slam_toolbox publishes /map latched (transient local).
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = msg;});

  // Top-priority twist_mux input; only used in AVOID_OBSTACLE / EMERGENCY_STOP.
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("autonomy/cmd_vel", rclcpp::QoS(10));
  state_pub_ = create_publisher<std_msgs::msg::String>("autonomy/state", rclcpp::QoS(10));

  start_explore_srv_ = create_service<std_srvs::srv::Trigger>(
    "autonomy/start_explore",
    std::bind(
      &AutonomousDecisionMaker::handle_start_explore, this,
      std::placeholders::_1, std::placeholders::_2));
  return_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "autonomy/return_home",
    std::bind(
      &AutonomousDecisionMaker::handle_return_home, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "autonomy/stop",
    std::bind(
      &AutonomousDecisionMaker::handle_stop, this,
      std::placeholders::_1, std::placeholders::_2));
  clear_emergency_srv_ = create_service<std_srvs::srv::Trigger>(
    "autonomy/clear_emergency",
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
  const auto sector = rover_autonomy::analyze_front_sector(*msg, frontal_half_rad_);
  obstacle_detected_ = sector.closest < obstacle_distance_;
  // Turn toward whichever side has more room, so the avoidance maneuver is a
  // little smarter than "always spin the same way."
  avoid_turn_direction_ = sector.roomier_side();
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

std::optional<std::pair<double, double>> AutonomousDecisionMaker::robot_position()
{
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
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
  if (new_state == State::EXPLORE && current_state_ != State::AVOID_OBSTACLE) {
    // A fresh mission gets a fresh chance at every frontier. (Coming back
    // from AVOID_OBSTACLE keeps the target, so its give-up timer keeps running.)
    blacklist_.clear();
    has_target_ = false;
    progress_area_ = 0.0;
    progress_time_ = now();
  }

  if (new_state == State::IDLE || new_state == State::EMERGENCY_STOP) {
    publish_cmd_vel(0.0, 0.0);
  }

  current_state_ = new_state;
}

void AutonomousDecisionMaker::tick_explore()
{
  // Done when the map stops growing. Once the rooms are mapped, the only
  // frontiers left are ones a 2D lidar can never resolve - e.g. the stretch
  // of outer wall hidden behind a couch, which leaks into the unknown outside
  // the house - and chasing them maps nothing new.
  if (map_) {
    const double area = known_free_area(*map_);
    if (area > progress_area_ + explore_min_progress_) {
      progress_area_ = area;
      progress_time_ = now();
    } else if ((now() - progress_time_).seconds() > explore_stall_timeout_) {
      RCLCPP_INFO(
        get_logger(), "Map hasn't grown in %.0fs (%.0f m^2 mapped) - exploration complete, "
        "returning home", explore_stall_timeout_, area);
      transition_to(State::RETURN_HOME);
      return;
    }
  }

  if (goal_active_) {
    if ((now() - target_started_time_).seconds() > explore_goal_timeout_) {
      RCLCPP_WARN(
        get_logger(), "Frontier (%.1f, %.1f) not reached in %.0fs, blacklisting it",
        target_x_, target_y_, explore_goal_timeout_);
      blacklist_.emplace_back(target_x_, target_y_);
      cancel_active_goal();
    } else if (map_ && (now() - last_frontier_check_).seconds() > 1.0) {
      // Once the lidar has seen past the target there's nothing left to
      // reveal there - don't finish driving to it, pick the next frontier.
      last_frontier_check_ = now();
      if (!frontier_near(*map_, target_x_, target_y_, 0.5)) {
        cancel_active_goal();
      } else {
        return;
      }
    } else {
      return;
    }
  }

  if (goal_finished_ && has_target_) {
    if (!goal_succeeded_) {
      RCLCPP_WARN(
        get_logger(), "Nav2 couldn't reach frontier (%.1f, %.1f), blacklisting it",
        target_x_, target_y_);
      blacklist_.emplace_back(target_x_, target_y_);
    } else if (map_ && frontier_near(*map_, target_x_, target_y_, 0.5)) {
      // Arrived and it's *still* unexplored (e.g. behind something the 2D
      // lidar can't see over) - going back won't help either.
      blacklist_.emplace_back(target_x_, target_y_);
    }
  }
  goal_finished_ = false;

  if (!send_best_frontier()) {
    RCLCPP_INFO(get_logger(), "No frontiers left - exploration complete, returning home");
    transition_to(State::RETURN_HOME);
  }
}

bool AutonomousDecisionMaker::send_best_frontier()
{
  const auto robot = robot_position();
  if (!map_ || !robot) {
    return true;  // nothing to decide on yet; not the same as "finished"
  }

  // explore_lite-style scoring: prefer frontiers that are close (cheap to
  // reach) and long (reveal a lot). Lower cost wins.
  const auto frontiers = find_frontiers(
    *map_, robot->first, robot->second, frontier_min_size_, frontier_min_goal_distance_,
    frontier_min_unknown_area_, frontier_min_clearance_);
  const Frontier * best = nullptr;
  double best_cost = 0.0;
  for (const auto & f : frontiers) {
    if (blacklisted(f.x, f.y)) {
      continue;
    }
    const double cost = frontier_distance_weight_ * f.distance - frontier_size_weight_ * f.size_m;
    if (!best || cost < best_cost) {
      best = &f;
      best_cost = cost;
    }
  }
  if (!best) {
    return false;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = map_frame_;
  goal.header.stamp = now();
  goal.pose.position.x = best->x;
  goal.pose.position.y = best->y;
  // Arrive facing away from the robot's current position, i.e. into the unknown.
  const double yaw = std::atan2(best->y - robot->second, best->x - robot->first);
  goal.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(
    get_logger(), "Exploring frontier at (%.1f, %.1f): %.1f m long, %.1f m away (%zu candidates)",
    best->x, best->y, best->size_m, best->distance, frontiers.size());
  // The give-up timer belongs to the frontier, not to the Nav2 goal: resending
  // the same frontier (e.g. after an AVOID_OBSTACLE interruption) keeps it.
  const bool same_target =
    has_target_ && std::hypot(best->x - target_x_, best->y - target_y_) < frontier_blacklist_radius_;
  if (!same_target) {
    target_started_time_ = now();
  }
  target_x_ = best->x;
  target_y_ = best->y;
  has_target_ = true;
  last_frontier_check_ = now();
  send_nav_goal(goal);
  return true;
}

bool AutonomousDecisionMaker::blacklisted(double x, double y) const
{
  return std::any_of(
    blacklist_.begin(), blacklist_.end(), [&](const auto & b) {
      return std::hypot(b.first - x, b.second - y) < frontier_blacklist_radius_;
    });
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

  const uint64_t seq = ++goal_seq_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, seq](const GoalHandleNav::SharedPtr & goal_handle) {
      nav_goal_response_callback(seq, goal_handle);
    };
  options.result_callback = [this, seq](const GoalHandleNav::WrappedResult & result) {
      nav_result_callback(seq, result);
    };

  goal_active_ = true;
  goal_finished_ = false;
  goal_succeeded_ = false;

  nav_client_->async_send_goal(goal_msg, options);
}

void AutonomousDecisionMaker::cancel_active_goal()
{
  if (current_goal_handle_) {
    nav_client_->async_cancel_goal(current_goal_handle_);
  }
  // If the goal was sent but Nav2 hasn't accepted it yet, there's no handle
  // to cancel - bumping the seq makes nav_goal_response_callback cancel it
  // as soon as it's accepted instead.
  ++goal_seq_;
  goal_active_ = false;
  current_goal_handle_.reset();
}

void AutonomousDecisionMaker::nav_goal_response_callback(
  uint64_t goal_seq, const GoalHandleNav::SharedPtr & goal_handle)
{
  if (goal_seq != goal_seq_) {
    // Cancelled before Nav2 accepted it; make sure it doesn't keep driving.
    if (goal_handle) {
      nav_client_->async_cancel_goal(goal_handle);
    }
    return;
  }
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Navigation goal was rejected by Nav2");
    goal_active_ = false;
    goal_finished_ = true;
    goal_succeeded_ = false;
    return;
  }
  current_goal_handle_ = goal_handle;
}

void AutonomousDecisionMaker::nav_result_callback(
  uint64_t goal_seq, const GoalHandleNav::WrappedResult & result)
{
  if (goal_seq != goal_seq_) {
    return;  // result of a goal we already cancelled/replaced
  }
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

}  // namespace rover_autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_autonomy::AutonomousDecisionMaker>());
  rclcpp::shutdown();
  return 0;
}
