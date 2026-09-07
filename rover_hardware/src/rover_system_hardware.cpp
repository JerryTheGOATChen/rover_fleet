#include "rover_hardware/rover_system_hardware.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rover_hardware
{

namespace
{
bool find_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, std::string & out)
{
  auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  out = it->second;
  return true;
}
}  // namespace

CallbackReturn RoverSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    std::string s;
    if (find_param(info_.hardware_parameters, "gpio_chip", s)) {gpio_chip_ = std::stoi(s);}
    if (!find_param(info_.hardware_parameters, "standby_pin", s)) {
      RCLCPP_ERROR(logger_, "Missing required hardware param 'standby_pin'");
      return CallbackReturn::ERROR;
    }
    standby_pin_ = std::stoi(s);

    if (find_param(info_.hardware_parameters, "pwm_frequency_hz", s)) {
      pwm_frequency_hz_ = std::stod(s);
    }
    if (find_param(info_.hardware_parameters, "max_wheel_angular_velocity", s)) {
      max_wheel_angular_velocity_ = std::stod(s);
    }
    if (find_param(info_.hardware_parameters, "velocity_kp", s)) {velocity_kp_ = std::stod(s);}
    if (find_param(info_.hardware_parameters, "velocity_ki", s)) {velocity_ki_ = std::stod(s);}

    const std::vector<std::pair<std::string, int *>> required_pins = {
      {"left_pwm_pin", &left_pwm_pin_},
      {"left_dir1_pin", &left_dir1_pin_},
      {"left_dir2_pin", &left_dir2_pin_},
      {"right_pwm_pin", &right_pwm_pin_},
      {"right_dir1_pin", &right_dir1_pin_},
      {"right_dir2_pin", &right_dir2_pin_},
    };
    for (const auto & [key, target] : required_pins) {
      if (!find_param(info_.hardware_parameters, key, s)) {
        RCLCPP_ERROR(logger_, "Missing required hardware param '%s'", key.c_str());
        return CallbackReturn::ERROR;
      }
      *target = std::stoi(s);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse hardware_parameters: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  left_pid_.kp = velocity_kp_;
  left_pid_.ki = velocity_ki_;
  right_pid_.kp = velocity_kp_;
  right_pid_.ki = velocity_ki_;

  wheels_.clear();
  wheels_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        logger_, "Joint '%s' must expose exactly one velocity command interface",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        logger_, "Joint '%s' must expose position and velocity state interfaces",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    WheelJoint wheel;
    wheel.name = joint.name;

    try {
      std::string side_str;
      if (!find_param(joint.parameters, "side", side_str)) {
        RCLCPP_ERROR(logger_, "Joint '%s' missing required param 'side'", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      if (side_str == "left") {
        wheel.side = Side::LEFT;
      } else if (side_str == "right") {
        wheel.side = Side::RIGHT;
      } else {
        RCLCPP_ERROR(
          logger_, "Joint '%s' param 'side' must be 'left' or 'right', got '%s'",
          joint.name.c_str(), side_str.c_str());
        return CallbackReturn::ERROR;
      }

      std::string s;
      if (!find_param(joint.parameters, "encoder_a_pin", s)) {
        RCLCPP_ERROR(logger_, "Joint '%s' missing 'encoder_a_pin'", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      wheel.encoder_a_pin = std::stoi(s);

      if (!find_param(joint.parameters, "encoder_b_pin", s)) {
        RCLCPP_ERROR(logger_, "Joint '%s' missing 'encoder_b_pin'", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      wheel.encoder_b_pin = std::stoi(s);

      if (!find_param(joint.parameters, "encoder_counts_per_rev", s)) {
        RCLCPP_ERROR(logger_, "Joint '%s' missing 'encoder_counts_per_rev'", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      wheel.counts_per_rev = std::stod(s);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to parse parameters for joint '%s': %s",
        joint.name.c_str(), ex.what());
      return CallbackReturn::ERROR;
    }

    wheels_.push_back(std::move(wheel));
  }

  RCLCPP_INFO(logger_, "Parsed hardware_parameters and %zu wheel joints", wheels_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_POSITION, &wheel.position);
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.command);
  }
  return interfaces;
}

CallbackReturn RoverSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  lgpio_handle_ = lgGpiochipOpen(gpio_chip_);
  if (lgpio_handle_ < 0) {
    RCLCPP_ERROR(
      logger_,
      "Failed to open /dev/gpiochip%d (error %d). Run `gpiodetect` and confirm which "
      "chip is labeled pinctrl-rp1 - it isn't always chip 0, this has changed across "
      "kernel versions.",
      gpio_chip_, lgpio_handle_);
    return CallbackReturn::ERROR;
  }

  if (lgGpioClaimOutput(lgpio_handle_, 0, standby_pin_, 0) < 0) {
    RCLCPP_ERROR(logger_, "Failed to claim standby_pin %d", standby_pin_);
    return CallbackReturn::ERROR;
  }

  if (!left_channel_.init(
      lgpio_handle_, left_pwm_pin_, left_dir1_pin_, left_dir2_pin_, pwm_frequency_hz_))
  {
    RCLCPP_ERROR(logger_, "Failed to claim GPIO for the left motor channel");
    return CallbackReturn::ERROR;
  }
  if (!right_channel_.init(
      lgpio_handle_, right_pwm_pin_, right_dir1_pin_, right_dir2_pin_, pwm_frequency_hz_))
  {
    RCLCPP_ERROR(logger_, "Failed to claim GPIO for the right motor channel");
    return CallbackReturn::ERROR;
  }

  for (auto & wheel : wheels_) {
    if (!wheel.encoder.init(lgpio_handle_, wheel.encoder_a_pin, wheel.encoder_b_pin)) {
      RCLCPP_ERROR(
        logger_, "Failed to claim encoder GPIO for joint '%s' (pins %d/%d)",
        wheel.name.c_str(), wheel.encoder_a_pin, wheel.encoder_b_pin);
      return CallbackReturn::ERROR;
    }
    wheel.last_ticks = wheel.encoder.count();
    wheel.position = 0.0;
    wheel.velocity = 0.0;
    wheel.command = 0.0;
  }

  RCLCPP_INFO(logger_, "Configured: gpiochip%d, %zu encoders claimed", gpio_chip_, wheels_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & wheel : wheels_) {
    wheel.encoder.close();
  }
  left_channel_.close();
  right_channel_.close();
  if (lgpio_handle_ >= 0) {
    lgGpioFree(lgpio_handle_, standby_pin_);
    lgGpiochipClose(lgpio_handle_);
    lgpio_handle_ = -1;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & wheel : wheels_) {
    wheel.command = 0.0;
  }
  left_channel_.stop();
  right_channel_.stop();
  left_pid_.reset();
  right_pid_.reset();

  // STBY high: this is the hardware-level "motors may move" gate, separate
  // from zeroing commands in software. Only asserted here, in on_activate -
  // see on_deactivate for the other half of this safety property.
  lgGpioWrite(lgpio_handle_, standby_pin_, 1);

  RCLCPP_INFO(logger_, "Activated: STBY high, motor channels enabled");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  left_channel_.stop();
  right_channel_.stop();
  // STBY low: hard, hardware-level disable of both H-bridge channels,
  // independent of whatever software state controller_manager is in.
  lgGpioWrite(lgpio_handle_, standby_pin_, 0);

  RCLCPP_INFO(logger_, "Deactivated: STBY low, motor channels disabled");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0) {
    return hardware_interface::return_type::OK;
  }

  for (auto & wheel : wheels_) {
    const int64_t ticks_now = wheel.encoder.count();
    const int64_t delta_ticks = ticks_now - wheel.last_ticks;
    wheel.last_ticks = ticks_now;

    const double delta_rad = (static_cast<double>(delta_ticks) / wheel.counts_per_rev) *
      2.0 * M_PI;

    wheel.velocity = delta_rad / dt;
    wheel.position += delta_rad;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Average the two wheels sharing each side rather than trusting a single
  // wheel's command/measurement - diff_drive_controller sends the same
  // commanded value to every wheel on a side by construction, but averaging
  // the *measured* velocity across both of that side's encoders is cheap
  // noise rejection.
  double left_cmd = 0.0, left_meas = 0.0;
  int left_n = 0;
  double right_cmd = 0.0, right_meas = 0.0;
  int right_n = 0;

  for (const auto & wheel : wheels_) {
    if (wheel.side == Side::LEFT) {
      left_cmd += wheel.command;
      left_meas += wheel.velocity;
      ++left_n;
    } else {
      right_cmd += wheel.command;
      right_meas += wheel.velocity;
      ++right_n;
    }
  }
  if (left_n > 0) {left_cmd /= left_n; left_meas /= left_n;}
  if (right_n > 0) {right_cmd /= right_n; right_meas /= right_n;}

  auto to_duty = [this, dt](double cmd, double meas, VelocityPid & pid) {
      double duty = cmd / max_wheel_angular_velocity_;  // feedforward
      if (dt > 0.0) {
        duty += pid.update(cmd - meas, dt);  // correction, zero by default (kp=ki=0)
      }
      return duty;
    };

  left_channel_.set_duty(to_duty(left_cmd, left_meas, left_pid_));
  right_channel_.set_duty(to_duty(right_cmd, right_meas, right_pid_));

  return hardware_interface::return_type::OK;
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::RoverSystemHardware, hardware_interface::SystemInterface)
