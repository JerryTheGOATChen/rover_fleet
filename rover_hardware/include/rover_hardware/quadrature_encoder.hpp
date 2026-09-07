#ifndef ROVER_HARDWARE_QUADRATURE_ENCODER_HPP_
#define ROVER_HARDWARE_QUADRATURE_ENCODER_HPP_

#include <atomic>
#include <cstdint>

extern "C" {
#include <lgpio.h>
}

namespace rover_hardware
{

/// Reads one quadrature encoder via lgpio edge alerts on channel A, using
/// channel B's instantaneous level to decide direction (X2 decoding: counts
/// both edges of A, not all four edges of a full X4 decode - half the
/// resolution of a full quadrature decoder, but far simpler and plenty for
/// wheel odometry given these motors' encoders are already fairly high
/// resolution after gearing).
///
/// If a wheel reports negative velocity while driving forward, the A/B
/// wiring (or this class's direction convention) is flipped for that motor -
/// swap the encoder_a_pin/encoder_b_pin parameters for that joint rather
/// than editing code.
class QuadratureEncoder
{
public:
  QuadratureEncoder() = default;
  ~QuadratureEncoder();

  QuadratureEncoder(const QuadratureEncoder &) = delete;
  QuadratureEncoder & operator=(const QuadratureEncoder &) = delete;

  /// Claims pin_a for edge alerts and pin_b for plain input. Returns false
  /// (and leaves the encoder inert) if either GPIO can't be claimed.
  bool init(int lgpio_handle, int pin_a, int pin_b);

  /// Signed tick count since init(). Thread-safe: the alert callback runs on
  /// an lgpio-internal thread, this just does an atomic load.
  int64_t count() const { return count_.load(std::memory_order_relaxed); }

  void close();

private:
  static void alert_callback(int num_alerts, lgGpioAlert_p events, void * user_data);
  void on_edge(int level_a);

  int handle_{-1};
  int pin_a_{-1};
  int pin_b_{-1};
  bool claimed_{false};
  std::atomic<int64_t> count_{0};
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE_QUADRATURE_ENCODER_HPP_
