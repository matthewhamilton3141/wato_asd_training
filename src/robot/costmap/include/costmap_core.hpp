#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Builds a robot-centred local costmap from a single LaserScan.
//
// Cells are -1 (unknown) until a ray passes through them (0, free) or ends on
// them (100, obstacle). Obstacles are then inflated with a linear falloff so
// the planner keeps its distance.
class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Must be called once before update(). Grid is width x height cells,
    // centred on the sensor frame.
    void configure(double resolution, int width, int height,
                   double inflation_radius, double min_range);

    // Rebuilds the grid from a scan. The result is available via grid().
    void update(const sensor_msgs::msg::LaserScan& scan);

    const nav_msgs::msg::OccupancyGrid& grid() const { return grid_; }

  private:
    // Marks free cells along the ray from the grid centre to (x1, y1) using
    // Bresenham's line algorithm. Does not touch the endpoint itself.
    void traceFree(int x1, int y1);

    // Traces free-only rays between adjacent beams so distant free space
    // isn't left as radial spokes of unknown cells.
    void fillBeamGaps(const sensor_msgs::msg::LaserScan& scan,
                      const std::vector<double>& beam_range);

    // Applies the inflation kernel around every obstacle cell in obstacles_.
    void inflate();

    bool inBounds(int x, int y) const {
      return x >= 0 && y >= 0 && x < width_ && y < height_;
    }
    int8_t& at(int x, int y) { return grid_.data[y * width_ + x]; }

    rclcpp::Logger logger_;

    double resolution_{0.1};
    int width_{0};
    int height_{0};
    double inflation_radius_{1.0};
    double min_range_{0.0};

    // Precomputed inflation kernel: (dx, dy, cost) for every offset inside the
    // inflation radius, sorted so it can be applied with a single max().
    struct KernelCell { int dx; int dy; int8_t cost; };
    std::vector<KernelCell> kernel_;

    std::vector<std::pair<int, int>> obstacles_;
    nav_msgs::msg::OccupancyGrid grid_;
};

}  

#endif  
