#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  const int obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 40);
  const double cost_weight = this->declare_parameter<double>("cost_weight", 3.0);
  const double snap_radius = this->declare_parameter<double>("snap_radius", 2.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  replan_timeout_ = this->declare_parameter<double>("replan_timeout", 5.0);
  const double timer_period = this->declare_parameter<double>("timer_period", 0.5);
  const std::string map_topic = this->declare_parameter<std::string>("map_topic", "/map");
  const std::string goal_topic = this->declare_parameter<std::string>("goal_topic", "/goal_point");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  const std::string path_topic = this->declare_parameter<std::string>("path_topic", "/path");

  planner_.configure(obstacle_threshold, cost_weight, snap_radius);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

  // Map is latched by map_memory, so match its durability.
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local(),
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period), std::bind(&PlannerNode::timerCallback, this));

  last_plan_time_ = this->now();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  // New information about the world: the current path may now cut through an
  // obstacle we hadn't seen yet, so replan.
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  planned_goal_.reset();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }

  if ((this->now() - last_plan_time_).seconds() > replan_timeout_) {
    RCLCPP_INFO(this->get_logger(), "Replanning (timeout)");
    planPath();
  }
}

bool PlannerNode::goalReached() const {
  if (!planned_goal_ || !odom_) return false;
  const double dx = planned_goal_->x - odom_->pose.pose.position.x;
  const double dy = planned_goal_->y - odom_->pose.pose.position.y;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!goal_ || !odom_ || !map_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan yet: missing %s%s%s",
                goal_ ? "" : "goal ", odom_ ? "" : "odom ", map_ ? "" : "map");
    return;
  }

  last_plan_time_ = this->now();
  auto path = planner_.plan(*map_,
                            odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                            goal_->point.x, goal_->point.y);
  if (!path) {
    RCLCPP_WARN(this->get_logger(), "No path to goal; stopping robot");
    publishEmptyPath();
    return;
  }

  path->header.stamp = odom_->header.stamp;
  for (auto& p : path->poses) p.header.stamp = path->header.stamp;
  planned_goal_ = path->poses.back().pose.position;
  path_pub_->publish(*path);
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path empty;
  empty.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  empty.header.stamp = odom_ ? rclcpp::Time(odom_->header.stamp) : this->now();
  path_pub_->publish(empty);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
