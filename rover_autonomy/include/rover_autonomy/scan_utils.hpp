#ifndef ROVER_AUTONOMY__SCAN_UTILS_HPP_
#define ROVER_AUTONOMY__SCAN_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace rover_autonomy
{

/// Summary of the lidar returns inside a frontal cone of +/- half_angle_rad
/// around the robot's +x axis. Shared by lidar_safety_node and
/// autonomous_decision_maker so both agree on what "obstacle ahead" means.
struct FrontSector
{
  double closest{std::numeric_limits<double>::infinity()};
  double left_avg{0.0};   // mean range of valid returns at angle >= 0 (ccw)
  double right_avg{0.0};  // mean range of valid returns at angle < 0 (cw)

  /// +1 = more room on the left (turn ccw), -1 = more room on the right.
  double roomier_side() const {return left_avg >= right_avg ? 1.0 : -1.0;}
};

inline FrontSector analyze_front_sector(
  const sensor_msgs::msg::LaserScan & scan, double half_angle_rad)
{
  FrontSector sector;
  double left_sum = 0.0;
  double right_sum = 0.0;
  int left_count = 0;
  int right_count = 0;

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::abs(angle) > half_angle_rad) {
      continue;
    }

    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    sector.closest = std::min(sector.closest, static_cast<double>(range));
    if (angle >= 0.0) {
      left_sum += range;
      ++left_count;
    } else {
      right_sum += range;
      ++right_count;
    }
  }

  sector.left_avg = left_count > 0 ? left_sum / left_count : 0.0;
  sector.right_avg = right_count > 0 ? right_sum / right_count : 0.0;
  return sector;
}

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__SCAN_UTILS_HPP_
