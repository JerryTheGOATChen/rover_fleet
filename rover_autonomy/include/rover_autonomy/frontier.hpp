#ifndef ROVER_AUTONOMY__FRONTIER_HPP_
#define ROVER_AUTONOMY__FRONTIER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace rover_autonomy
{

/// A frontier: a connected stretch of known-free cells that border unknown
/// space. Driving to one reveals what's behind it.
struct Frontier
{
  double x{0.0};         // goal point in the map frame: a frontier cell (always
  double y{0.0};         //   free) near the cluster's centroid - see find_frontiers
  double size_m{0.0};    // length of the frontier (cells * resolution)
  double distance{0.0};  // straight-line distance from the robot
};

namespace detail
{
inline bool is_free(int8_t v) {return v >= 0 && v < 50;}
inline bool is_occupied(int8_t v) {return v >= 50;}

/// True if no occupied cell lies within r_cells of (cx, cy).
inline bool has_clearance(const nav_msgs::msg::OccupancyGrid & map, int cx, int cy, int r_cells)
{
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (dx * dx + dy * dy <= r_cells * r_cells && nx >= 0 && ny >= 0 && nx < w && ny < h &&
        is_occupied(map.data[ny * w + nx]))
      {
        return false;
      }
    }
  }
  return true;
}
inline bool is_unknown(int8_t v) {return v < 0;}

/// Free cell with at least one unknown 4-neighbour.
inline bool is_frontier_cell(const nav_msgs::msg::OccupancyGrid & map, int cx, int cy)
{
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  if (!is_free(map.data[cy * w + cx])) {
    return false;
  }
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    const int nx = cx + dx[k];
    const int ny = cy + dy[k];
    if (nx >= 0 && ny >= 0 && nx < w && ny < h && is_unknown(map.data[ny * w + nx])) {
      return true;
    }
  }
  return false;
}
}  // namespace detail

/// Finds all frontiers of at least min_size_m in the map, clustering frontier
/// cells by 8-connectivity. Map values: -1 unknown, 0..49 free, 50+ occupied.
///
/// Each frontier's goal is its cell nearest the cluster centroid among cells
/// at least min_goal_distance_m from the robot and min_clearance_m from any
/// obstacle. The clearance keeps goals out of nooks (between furniture and a
/// wall) where a rectangular robot can't turn around and gets stuck; a
/// frontier with no such cell is skipped. The distance floor matters:
/// early on, the known area is a small blob whose whole edge is one ring
/// frontier, and a ring's centroid is the robot itself - a goal there is
/// "reached" instantly, the robot never moves, and SLAM (which only adds
/// scans after the robot moves) never grows the map.
///
/// Frontiers opening onto less than min_unknown_area_m2 of unknown space are
/// dropped: those are shadows behind or under furniture that a 2D lidar can
/// never see into (a bed's footprint, the gap behind a plant), not unexplored
/// rooms. Without this the robot spends minutes chasing them once the real
/// exploring is done.
inline std::vector<Frontier> find_frontiers(
  const nav_msgs::msg::OccupancyGrid & map, double robot_x, double robot_y,
  double min_size_m, double min_goal_distance_m, double min_unknown_area_m2,
  double min_clearance_m)
{
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;

  std::vector<Frontier> frontiers;
  std::vector<uint8_t> visited(static_cast<std::size_t>(w) * h, 0);
  std::vector<int> unknown_seen(static_cast<std::size_t>(w) * h, -1);  // cluster id that counted it
  const std::size_t unknown_cap =
    static_cast<std::size_t>(std::ceil(min_unknown_area_m2 / (res * res)));
  const int d4x[4] = {1, -1, 0, 0};
  const int d4y[4] = {0, 0, 1, -1};
  const int clearance_cells = static_cast<int>(std::ceil(min_clearance_m / res));

  for (int start = 0; start < w * h; ++start) {
    if (visited[start] || !detail::is_frontier_cell(map, start % w, start / w)) {
      continue;
    }

    // Flood-fill this cluster.
    std::vector<int> cells;
    std::queue<int> open;
    open.push(start);
    visited[start] = 1;
    while (!open.empty()) {
      const int c = open.front();
      open.pop();
      cells.push_back(c);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = c % w + dx;
          const int ny = c / w + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
            continue;
          }
          const int n = ny * w + nx;
          if (!visited[n] && detail::is_frontier_cell(map, nx, ny)) {
            visited[n] = 1;
            open.push(n);
          }
        }
      }
    }

    const double size_m = static_cast<double>(cells.size()) * res;
    if (size_m < min_size_m) {
      continue;  // lidar speckle, not a real opening
    }

    // Measure the unknown region behind this frontier, stopping once it's
    // clearly big enough to be worth exploring.
    const int id = start;
    std::size_t unknown_count = 0;
    std::queue<int> unk;
    for (int c : cells) {
      for (int k = 0; k < 4; ++k) {
        const int nx = c % w + d4x[k];
        const int ny = c / w + d4y[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        const int n = ny * w + nx;
        if (unknown_seen[n] != id && detail::is_unknown(map.data[n])) {
          unknown_seen[n] = id;
          unk.push(n);
        }
      }
    }
    while (!unk.empty() && unknown_count < unknown_cap) {
      const int c = unk.front();
      unk.pop();
      ++unknown_count;
      for (int k = 0; k < 4; ++k) {
        const int nx = c % w + d4x[k];
        const int ny = c / w + d4y[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        const int n = ny * w + nx;
        if (unknown_seen[n] != id && detail::is_unknown(map.data[n])) {
          unknown_seen[n] = id;
          unk.push(n);
        }
      }
    }
    const double unknown_m2 = static_cast<double>(unknown_count) * res * res;
    if (unknown_m2 < min_unknown_area_m2) {
      continue;  // a shadow pocket, not unexplored space
    }

    double mx = 0.0, my = 0.0;
    for (int c : cells) {
      mx += c % w;
      my += c / w;
    }
    mx /= static_cast<double>(cells.size());
    my /= static_cast<double>(cells.size());

    // The centroid itself can land in unknown space or a wall (or on the
    // robot, for a ring), so aim for an actual frontier cell near it: try
    // cells in order of closeness to the centroid, take the first usable one.
    std::vector<std::pair<double, int>> by_centroid;
    by_centroid.reserve(cells.size());
    for (int c : cells) {
      by_centroid.emplace_back(std::pow(c % w - mx, 2) + std::pow(c / w - my, 2), c);
    }
    std::sort(by_centroid.begin(), by_centroid.end());
    int best = -1;
    for (const auto & [d2, c] : by_centroid) {
      const double wx = ox + (c % w + 0.5) * res;
      const double wy = oy + (c / w + 0.5) * res;
      if (std::hypot(wx - robot_x, wy - robot_y) >= min_goal_distance_m &&
        detail::has_clearance(map, c % w, c / w, clearance_cells))
      {
        best = c;
        break;
      }
    }
    if (best < 0) {
      continue;  // all within reach already, or wedged in a nook
    }

    Frontier f;
    f.x = ox + (best % w + 0.5) * res;
    f.y = oy + (best / w + 0.5) * res;
    f.size_m = size_m;
    f.distance = std::hypot(f.x - robot_x, f.y - robot_y);
    frontiers.push_back(f);
  }
  return frontiers;
}

/// True if any frontier cell lies within radius_m of (x, y) - i.e. a goal
/// placed there still has something left to reveal.
inline bool frontier_near(
  const nav_msgs::msg::OccupancyGrid & map, double x, double y, double radius_m)
{
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const int cx = static_cast<int>((x - map.info.origin.position.x) / res);
  const int cy = static_cast<int>((y - map.info.origin.position.y) / res);
  const int r = static_cast<int>(std::ceil(radius_m / res));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (nx >= 0 && ny >= 0 && nx < w && ny < h && dx * dx + dy * dy <= r * r &&
        detail::is_frontier_cell(map, nx, ny))
      {
        return true;
      }
    }
  }
  return false;
}

/// Known-free area of the map in m^2 - the exploration progress measure.
inline double known_free_area(const nav_msgs::msg::OccupancyGrid & map)
{
  std::size_t n = 0;
  for (int8_t v : map.data) {
    n += detail::is_free(v) ? 1 : 0;
  }
  return static_cast<double>(n) * map.info.resolution * map.info.resolution;
}

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__FRONTIER_HPP_
