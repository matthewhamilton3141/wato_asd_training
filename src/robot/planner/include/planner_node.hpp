#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Runs A* from the current pose to the current goal and publishes the result.
    void planPath();
    void publishEmptyPath();
    bool goalReached() const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double goal_tolerance_{0.5};
    double replan_timeout_{5.0};

    State state_{State::WAITING_FOR_GOAL};
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::optional<geometry_msgs::msg::PointStamped> goal_;
    // End of the last published path (the goal after snapping off obstacles).
    std::optional<geometry_msgs::msg::Point> planned_goal_;
    std::optional<nav_msgs::msg::Odometry> odom_;
    rclcpp::Time last_plan_time_;
};

#endif 
