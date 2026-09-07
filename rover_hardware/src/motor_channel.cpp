#include "rover_hardware/motor_channel.hpp"

#include <algorithm>

namespace rover_hardware
{

MotorChannel::~MotorChannel()
{
  close();
}

bool MotorChannel::init(
  int lgpio_handle, int pwm_pin, int dir1_pin, int dir2_pin, double pwm_frequency_hz)
{
  handle_ = lgpio_handle;
  pwm_pin_ = pwm_pin;
  dir1_pin_ = dir1_pin;
  dir2_pin_ = dir2_pin;
  pwm_frequency_hz_ = pwm_frequency_hz;

  if (lgGpioClaimOutput(handle_, 0, dir1_pin_, 0) < 0) {
    return false;
  }
  if (lgGpioClaimOutput(handle_, 0, dir2_pin_, 0) < 0) {
    lgGpioFree(handle_, dir1_pin_);
    return false;
  }
  if (lgGpioClaimOutput(handle_, 0, pwm_pin_, 0) < 0) {
    lgGpioFree(handle_, dir1_pin_);
    lgGpioFree(handle_, dir2_pin_);
    return false;
  }

  claimed_ = true;
  return true;
}

void MotorChannel::set_duty(double duty)
{
  duty = std::clamp(duty, -1.0, 1.0);

  if (duty > 0.0) {
    lgGpioWrite(handle_, dir1_pin_, 1);
    lgGpioWrite(handle_, dir2_pin_, 0);
  } else if (duty < 0.0) {
    lgGpioWrite(handle_, dir1_pin_, 0);
    lgGpioWrite(handle_, dir2_pin_, 1);
  } else {
    // Coast rather than brake at zero command - simpler and safer as a
    // default. Drive both direction pins high instead if you want the
    // TB6612FNG's short-brake behavior here.
    lgGpioWrite(handle_, dir1_pin_, 0);
    lgGpioWrite(handle_, dir2_pin_, 0);
  }

  const double duty_percent = std::abs(duty) * 100.0;
  // cycles=0: run indefinitely until the next lgTxPwm call changes it.
  lgTxPwm(handle_, pwm_pin_, static_cast<float>(pwm_frequency_hz_),
    static_cast<float>(duty_percent), 0, 0);
}

void MotorChannel::stop()
{
  set_duty(0.0);
}

void MotorChannel::close()
{
  if (!claimed_) {
    return;
  }
  lgTxPwm(handle_, pwm_pin_, 0.0f, 0.0f, 0, 0);
  lgGpioWrite(handle_, dir1_pin_, 0);
  lgGpioWrite(handle_, dir2_pin_, 0);
  lgGpioFree(handle_, pwm_pin_);
  lgGpioFree(handle_, dir1_pin_);
  lgGpioFree(handle_, dir2_pin_);
  claimed_ = false;
}

}  // namespace rover_hardware
