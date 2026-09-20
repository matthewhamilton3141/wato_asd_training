#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// Pure pursuit path tracker for a differential-drive robot.
class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double goal_tolerance,
                   double linear_speed, double max_angular_speed,
                   double turn_in_place_angle);

    void setPath(const nav_msgs::msg::Path& path);
    bool hasPath() const { return !path_.poses.empty(); }

    // One control step. Returns a zero twist when there is no path or the
    // end of the path has been reached.
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Odometry& odom);

  private:
    // Index of the path pose nearest the robot.
    size_t closestIndex(double x, double y) const;
    // First pose at least lookahead_distance_ ahead of `from`, or the last pose.
    geometry_msgs::msg::Point lookaheadPoint(size_t from, double x, double y) const;

    rclcpp::Logger logger_;

    double lookahead_distance_{1.5};
    double goal_tolerance_{0.2};
    double linear_speed_{0.5};
    double max_angular_speed_{1.0};
    double turn_in_place_angle_{1.2};

    nav_msgs::msg::Path path_;
};

} 

#endif 
