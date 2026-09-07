#ifndef ROVER_HARDWARE_MOTOR_CHANNEL_HPP_
#define ROVER_HARDWARE_MOTOR_CHANNEL_HPP_

extern "C" {
#include <lgpio.h>
}

namespace rover_hardware
{

/// One TB6612FNG driver channel: one PWM pin (speed) and two direction pins
/// (IN1/IN2 - forward, reverse, or coast). Does not touch the shared STBY
/// pin; that's driven once at the RoverSystemHardware level so on_activate/
/// on_deactivate can enable/disable both channels together.
class MotorChannel
{
public:
  MotorChannel() = default;
  ~MotorChannel();

  MotorChannel(const MotorChannel &) = delete;
  MotorChannel & operator=(const MotorChannel &) = delete;

  bool init(int lgpio_handle, int pwm_pin, int dir1_pin, int dir2_pin, double pwm_frequency_hz);

  /// duty in [-1.0, 1.0]: sign is direction, magnitude is duty cycle
  /// fraction. Values outside that range are clamped.
  void set_duty(double duty);

  /// Coasts the motor (both direction pins low, PWM off) without releasing
  /// the GPIO claims - use during on_deactivate, not on_cleanup.
  void stop();

  void close();

private:
  int handle_{-1};
  int pwm_pin_{-1};
  int dir1_pin_{-1};
  int dir2_pin_{-1};
  double pwm_frequency_hz_{1000.0};
  bool claimed_{false};
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE_MOTOR_CHANNEL_HPP_
