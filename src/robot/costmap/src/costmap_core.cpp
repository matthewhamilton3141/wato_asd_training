#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>

namespace robot
{

namespace {
constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kLethal = 100;
}

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height,
                            double inflation_radius, double min_range)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  inflation_radius_ = inflation_radius;
  min_range_ = min_range;

  grid_.info.resolution = resolution_;
  grid_.info.width = width_;
  grid_.info.height = height_;
  // Grid is centred on the sensor, so the origin (bottom-left corner) sits
  // half a grid behind and to the right of it.
  grid_.info.origin.position.x = -0.5 * width_ * resolution_;
  grid_.info.origin.position.y = -0.5 * height_ * resolution_;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(width_ * height_, kUnknown);

  // Build the inflation kernel once. cost = 100 * (1 - d / r), so an obstacle
  // cell is 100 and it decays linearly to 0 at the inflation radius.
  kernel_.clear();
  const int r_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const double d = std::hypot(dx, dy) * resolution_;
      if (d > inflation_radius_) continue;
      const double c = kLethal * (1.0 - d / inflation_radius_);
      kernel_.push_back({dx, dy, static_cast<int8_t>(std::lround(c))});
    }
  }

  RCLCPP_INFO(logger_, "Costmap configured: %dx%d cells @ %.2fm, inflation %.2fm (%zu kernel cells)",
              width_, height_, resolution_, inflation_radius_, kernel_.size());
}

void CostmapCore::update(const sensor_msgs::msg::LaserScan& scan)
{
  std::fill(grid_.data.begin(), grid_.data.end(), kUnknown);
  obstacles_.clear();

  grid_.header = scan.header;
  grid_.info.map_load_time = scan.header.stamp;

  const int cx = width_ / 2;
  const int cy = height_ / 2;
  const double max_range = std::min<double>(scan.range_max, 0.5 * std::min(width_, height_) * resolution_);

  // Effective range of each beam after clamping, for the gap fill below.
  std::vector<double> beam_range(scan.ranges.size(), -1.0);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    double r = scan.ranges[i];

    // Rays that return nothing (inf / nan / beyond range_max) still tell us
    // the space up to max_range is free. Returns below range_min are the
    // sensor saying "too close to measure", so say nothing about them.
    bool hit = true;
    if (!std::isfinite(r) || r >= scan.range_max) {
      r = max_range;
      hit = false;
    } else if (r < scan.range_min || r < min_range_) {
      continue;
    } else if (r > max_range) {
      r = max_range;
      hit = false;
    }

    beam_range[i] = r;

    const double lx = r * std::cos(angle);
    const double ly = r * std::sin(angle);
    const int gx = cx + static_cast<int>(std::floor(lx / resolution_));
    const int gy = cy + static_cast<int>(std::floor(ly / resolution_));

    traceFree(gx, gy);
    if (hit && inBounds(gx, gy)) {
      at(gx, gy) = kLethal;
      obstacles_.emplace_back(gx, gy);
    } else if (inBounds(gx, gy) && at(gx, gy) == kUnknown) {
      at(gx, gy) = kFree;
    }
  }

  fillBeamGaps(scan, beam_range);
  inflate();
}

void CostmapCore::fillBeamGaps(const sensor_msgs::msg::LaserScan& scan,
                               const std::vector<double>& beam_range)
{
  // Adjacent beams diverge by r * angle_increment, which exceeds one cell
  // well before the lidar's max range and leaves radial spokes of unknown
  // cells. The wedge between two beams is free out to the nearer return
  // (obstacles are far bigger than the beam spacing), so trace extra
  // free-only rays through each wedge, dense enough to touch every cell.
  const int cx = width_ / 2;
  const int cy = height_ / 2;
  for (size_t i = 0; i + 1 < beam_range.size(); ++i) {
    const double r = std::min(beam_range[i], beam_range[i + 1]);
    if (r <= 0.0) continue;  // one of the beams was discarded

    const int subrays = static_cast<int>(std::ceil(r * scan.angle_increment / resolution_));
    const double a0 = scan.angle_min + i * scan.angle_increment;
    for (int k = 1; k < subrays; ++k) {
      const double angle = a0 + scan.angle_increment * k / subrays;
      const int gx = cx + static_cast<int>(std::floor(r * std::cos(angle) / resolution_));
      const int gy = cy + static_cast<int>(std::floor(r * std::sin(angle) / resolution_));
      traceFree(gx, gy);
    }
  }
}

void CostmapCore::traceFree(int x1, int y1)
{
  int x0 = width_ / 2;
  int y0 = height_ / 2;
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (x0 != x1 || y0 != y1) {
    if (!inBounds(x0, y0)) return;
    if (at(x0, y0) == kUnknown) at(x0, y0) = kFree;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void CostmapCore::inflate()
{
  for (const auto& [ox, oy] : obstacles_) {
    for (const auto& k : kernel_) {
      const int x = ox + k.dx;
      const int y = oy + k.dy;
      if (!inBounds(x, y)) continue;
      int8_t& cell = at(x, y);
      // Inflation overrides free/unknown but never lowers an existing cost.
      if (cell < k.cost) cell = k.cost;
    }
  }
}

}
