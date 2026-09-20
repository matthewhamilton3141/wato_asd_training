#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Accumulates robot-centred local costmaps into one fixed global map.
//
// Fusion walks every global cell under the costmap's footprint, projects its
// centre into the costmap frame and samples there. Sampling in that direction
// (global -> local) means a rotated costmap never leaves gaps in the global
// map. Known costs are merged with max() since the world is static, so an
// obstacle that drops out of view (occlusion) is never erased.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Global map is width x height cells at `resolution`, with its bottom-left
    // corner at (origin_x, origin_y) in `frame_id`.
    void configure(double resolution, int width, int height,
                   double origin_x, double origin_y, const std::string& frame_id);

    // Fuse a local costmap taken at robot pose (x, y, yaw) in the global frame.
    void integrate(const nav_msgs::msg::OccupancyGrid& costmap,
                   double x, double y, double yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }
    nav_msgs::msg::OccupancyGrid& map() { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}  

#endif  
