#include "map_memory_core.hpp"

#include <cmath>
#include <algorithm>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::configure(double resolution, int width, int height,
                              double origin_x, double origin_y, const std::string& frame_id)
{
  map_.header.frame_id = frame_id;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);

  RCLCPP_INFO(logger_, "Global map configured: %dx%d cells @ %.2fm, origin (%.1f, %.1f) in %s",
              width, height, resolution, origin_x, origin_y, frame_id.c_str());
}

void MapMemoryCore::integrate(const nav_msgs::msg::OccupancyGrid& costmap,
                              double x, double y, double yaw)
{
  const double gres = map_.info.resolution;
  const double gox = map_.info.origin.position.x;
  const double goy = map_.info.origin.position.y;
  const int gw = map_.info.width;
  const int gh = map_.info.height;

  const double lres = costmap.info.resolution;
  const double lox = costmap.info.origin.position.x;
  const double loy = costmap.info.origin.position.y;
  const int lw = costmap.info.width;
  const int lh = costmap.info.height;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // Bounding box of the (rotated) costmap in global cell coordinates, so we
  // only visit global cells that could possibly be covered.
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  for (int corner = 0; corner < 4; ++corner) {
    const double lx = lox + ((corner & 1) ? lw * lres : 0.0);
    const double ly = loy + ((corner & 2) ? lh * lres : 0.0);
    const double wx = x + c * lx - s * ly;
    const double wy = y + s * lx + c * ly;
    min_x = std::min(min_x, wx); max_x = std::max(max_x, wx);
    min_y = std::min(min_y, wy); max_y = std::max(max_y, wy);
  }
  const int gx0 = std::max(0, static_cast<int>(std::floor((min_x - gox) / gres)));
  const int gy0 = std::max(0, static_cast<int>(std::floor((min_y - goy) / gres)));
  const int gx1 = std::min(gw - 1, static_cast<int>(std::floor((max_x - gox) / gres)));
  const int gy1 = std::min(gh - 1, static_cast<int>(std::floor((max_y - goy) / gres)));

  size_t updated = 0;
  for (int gy = gy0; gy <= gy1; ++gy) {
    const double wy = goy + (gy + 0.5) * gres;
    for (int gx = gx0; gx <= gx1; ++gx) {
      const double wx = gox + (gx + 0.5) * gres;

      // World -> costmap frame (inverse of the robot pose).
      const double dx = wx - x;
      const double dy = wy - y;
      const double lx =  c * dx + s * dy;
      const double ly = -s * dx + c * dy;

      const int cx = static_cast<int>(std::floor((lx - lox) / lres));
      const int cy = static_cast<int>(std::floor((ly - loy) / lres));
      if (cx < 0 || cy < 0 || cx >= lw || cy >= lh) continue;

      const int8_t local = costmap.data[cy * lw + cx];
      if (local < 0) continue;  // unknown in this scan, keep what we had

      int8_t& global = map_.data[gy * gw + gx];
      if (local > global) { global = local; ++updated; }
    }
  }

  RCLCPP_DEBUG(logger_, "Integrated costmap at (%.2f, %.2f, %.2f rad): %zu cells raised",
               x, y, yaw, updated);
}

} 
