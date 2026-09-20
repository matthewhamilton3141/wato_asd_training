#include "control_core.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace robot
{

namespace {
double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void ControlCore::configure(double lookahead_distance, double goal_tolerance,
                            double linear_speed, double max_angular_speed,
                            double turn_in_place_angle)
{
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
  turn_in_place_angle_ = turn_in_place_angle;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path)
{
  path_ = path;
}

size_t ControlCore::closestIndex(double x, double y) const
{
  size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    const double d = std::hypot(p.x - x, p.y - y);
    if (d < best_d) { best_d = d; best = i; }
  }
  return best;
}

geometry_msgs::msg::Point ControlCore::lookaheadPoint(size_t from, double x, double y) const
{
  for (size_t i = from; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    if (std::hypot(p.x - x, p.y - y) >= lookahead_distance_) return p;
  }
  return path_.poses.back().pose.position;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Odometry& odom)
{
  geometry_msgs::msg::Twist cmd;
  if (path_.poses.empty()) return cmd;

  const double x = odom.pose.pose.position.x;
  const double y = odom.pose.pose.position.y;
  const double yaw = yawFromQuat(odom.pose.pose.orientation);

  const auto& goal = path_.poses.back().pose.position;
  const double dist_to_goal = std::hypot(goal.x - x, goal.y - y);
  if (dist_to_goal < goal_tolerance_) {
    return cmd;  // arrived: hold still
  }

  const size_t closest = closestIndex(x, y);
  const auto target = lookaheadPoint(closest, x, y);

  const double dx = target.x - x;
  const double dy = target.y - y;
  const double L = std::hypot(dx, dy);
  const double alpha = wrapAngle(std::atan2(dy, dx) - yaw);

  if (std::fabs(alpha) > turn_in_place_angle_) {
    // Target is well off to the side or behind: spin toward it first.
    cmd.angular.z = std::copysign(max_angular_speed_, alpha);
    return cmd;
  }

  // Pure pursuit: steer along the arc through the lookahead point.
  // curvature = 2 sin(alpha) / L, so omega = v * curvature.
  const double curvature = (L > 1e-3) ? 2.0 * std::sin(alpha) / L : 0.0;

  // Slow down as the heading error grows and as we approach the goal.
  double v = linear_speed_ * std::max(0.2, 1.0 - std::fabs(alpha) / turn_in_place_angle_);
  v = std::min(v, std::max(0.15, dist_to_goal));

  double w = v * curvature;
  if (std::fabs(w) > max_angular_speed_) {
    // Preserve the arc by scaling both down instead of clipping just omega.
    const double scale = max_angular_speed_ / std::fabs(w);
    v *= scale;
    w *= scale;
  }

  cmd.linear.x = v;
  cmd.angular.z = w;
  return cmd;
}

}  
