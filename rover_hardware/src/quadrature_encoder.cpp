#include "rover_hardware/quadrature_encoder.hpp"

namespace rover_hardware
{

QuadratureEncoder::~QuadratureEncoder()
{
  close();
}

bool QuadratureEncoder::init(int lgpio_handle, int pin_a, int pin_b)
{
  handle_ = lgpio_handle;
  pin_a_ = pin_a;
  pin_b_ = pin_b;

  if (lgGpioClaimInput(handle_, 0, pin_b_) < 0) {
    return false;
  }

  // LG_BOTH_EDGES + notifyHandle=-1: deliver events to the callback we
  // register below instead of a separate notification pipe.
  if (lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_a_, -1) < 0) {
    lgGpioFree(handle_, pin_b_);
    return false;
  }

  lgGpioSetAlertsFunc(handle_, pin_a_, &QuadratureEncoder::alert_callback, this);

  claimed_ = true;
  return true;
}

void QuadratureEncoder::close()
{
  if (!claimed_) {
    return;
  }
  lgGpioFree(handle_, pin_a_);
  lgGpioFree(handle_, pin_b_);
  claimed_ = false;
}

void QuadratureEncoder::alert_callback(int num_alerts, lgGpioAlert_p events, void * user_data)
{
  auto * self = static_cast<QuadratureEncoder *>(user_data);
  for (int i = 0; i < num_alerts; ++i) {
    self->on_edge(events[i].report.level);
  }
}

void QuadratureEncoder::on_edge(int level_a)
{
  // X2 decode: on every edge of A, compare A's new level to B's current
  // level. Same -> one direction, different -> the other. Direction
  // convention is arbitrary here; if it's backwards on your wheel, swap
  // encoder_a_pin/encoder_b_pin in the URDF rather than touching this code.
  const int level_b = lgGpioRead(handle_, pin_b_);
  if (level_a == level_b) {
    count_.fetch_add(1, std::memory_order_relaxed);
  } else {
    count_.fetch_sub(1, std::memory_order_relaxed);
  }
}

}  // namespace rover_hardware
