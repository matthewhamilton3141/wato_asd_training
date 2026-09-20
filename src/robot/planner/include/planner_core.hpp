#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// A 2D grid cell index.
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return (x != other.x || y != other.y); }
};

// Hash function for CellIndex so it can be used in std::unordered_map.
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set.
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score).
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

// Cost-aware 8-connected A* over an OccupancyGrid.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Cells with cost >= obstacle_threshold are impassable. Cells below it add
    // cost_weight * (cost / 100) to the step length, so paths prefer open
    // space. snap_radius is how far (m) a start/goal inside an obstacle may be
    // moved to the nearest free cell.
    void configure(int obstacle_threshold, double cost_weight, double snap_radius);

    // Plan from (sx, sy) to (gx, gy), both in the map frame. Returns a path
    // in map-frame metres, or nullopt if none exists. The first pose is the
    // exact start and the last pose is the exact goal.
    std::optional<nav_msgs::msg::Path> plan(const nav_msgs::msg::OccupancyGrid& map,
                                            double sx, double sy, double gx, double gy);

  private:
    bool worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& out) const;
    void cellToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c, double& wx, double& wy) const;
    int8_t costAt(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const;
    bool isFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const;

    // Nearest free cell to `c` within snap_radius_, searched breadth-first.
    std::optional<CellIndex> nearestFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const;

    rclcpp::Logger logger_;
    int obstacle_threshold_{40};
    double cost_weight_{3.0};
    double snap_radius_{2.0};
};

}  

#endif  
