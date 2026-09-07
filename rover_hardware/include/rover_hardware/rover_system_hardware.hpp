#ifndef ROVER_HARDWARE_ROVER_SYSTEM_HARDWARE_HPP_
#define ROVER_HARDWARE_ROVER_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rover_hardware/motor_channel.hpp"
#include "rover_hardware/quadrature_encoder.hpp"

extern "C" {
#include <lgpio.h>
}

namespace rover_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class Side { LEFT, RIGHT };

/// Minimal proportional-integral controller. Defaults to kp=ki=0 (pure
/// open-loop feedforward from commanded rad/s to PWM duty) on purpose - see
/// the README for why zero gain is the deliberate, safe starting point on
/// real hardware rather than a guessed tuning.
struct VelocityPid
{
  double kp{0.0};
  double ki{0.0};
  double integral_clamp{1.0};
  double integral{0.0};

  double update(double error, double dt)
  {
    integral += error * dt;
    if (integral > integral_clamp) {integral = integral_clamp;}
    if (integral < -integral_clamp) {integral = -integral_clamp;}
    return kp * error + ki * integral;
  }

  void reset() {integral = 0.0;}
};

struct WheelJoint
{
  std::string name;
  Side side{Side::LEFT};
  int encoder_a_pin{-1};
  int encoder_b_pin{-1};
  double counts_per_rev{1.0};

  QuadratureEncoder encoder;
  int64_t last_ticks{0};

  double position{0.0};   // radians, continuous
  double velocity{0.0};   // rad/s
  double command{0.0};    // rad/s, written by diff_drive_controller
};

/// ros2_control hardware interface for the physical rover: two TB6612FNG
/// channels (left side, right side - wheel1_joint/wheel4_joint share the
/// right channel, wheel2_joint/wheel3_joint share the left channel, matching
/// config/controllers.yaml's wheel groupings) and one quadrature encoder per
/// wheel joint, all via lgpio on Raspberry Pi 5's RP1 gpiochip.
///
/// Exposes position + velocity state interfaces and a velocity command
/// interface per joint, so it drops in under diff_drive_controller /
/// joint_state_broadcaster exactly like the Gazebo DiffDrive plugin does in
/// simulation - same controllers.yaml, same downstream SLAM/Nav2 config.
class RoverSystemHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- hardware-level parameters (from <ros2_control><hardware><param>) ---
  int gpio_chip_{0};
  int standby_pin_{-1};
  double pwm_frequency_hz_{1000.0};
  double max_wheel_angular_velocity_{10.0};  // rad/s at 100% duty - calibrate this
  double velocity_kp_{0.0};
  double velocity_ki_{0.0};

  int left_pwm_pin_{-1};
  int left_dir1_pin_{-1};
  int left_dir2_pin_{-1};
  int right_pwm_pin_{-1};
  int right_dir1_pin_{-1};
  int right_dir2_pin_{-1};

  int lgpio_handle_{-1};
  MotorChannel left_channel_;
  MotorChannel right_channel_;
  VelocityPid left_pid_;
  VelocityPid right_pid_;

  std::vector<WheelJoint> wheels_;

  rclcpp::Logger logger_{rclcpp::get_logger("rover_system_hardware")};
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE_ROVER_SYSTEM_HARDWARE_HPP_
