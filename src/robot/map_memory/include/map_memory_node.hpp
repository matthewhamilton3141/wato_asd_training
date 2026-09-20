#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Odometry sample closest in time to `stamp`, if one is buffered.
    std::optional<nav_msgs::msg::Odometry> odomAt(const rclcpp::Time& stamp) const;

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double update_distance_{1.5};

    // Recent odometry so a costmap can be paired with the pose at scan time
    // rather than whatever pose arrived last.
    std::deque<nav_msgs::msg::Odometry> odom_buffer_;

    // Latest costmap plus the pose it was taken from.
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    std::optional<nav_msgs::msg::Odometry> latest_costmap_odom_;

    // Where the robot was when the map was last updated.
    bool have_last_update_pose_{false};
    double last_x_{0.0};
    double last_y_{0.0};
    bool should_update_{true};
};

#endif 
