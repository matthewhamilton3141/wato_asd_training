#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  const double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.5);
  const double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.2);
  const double linear_speed = this->declare_parameter<double>("linear_speed", 0.5);
  const double max_angular_speed = this->declare_parameter<double>("max_angular_speed", 1.0);
  const double turn_in_place_angle = this->declare_parameter<double>("turn_in_place_angle", 1.2);
  const double control_period = this->declare_parameter<double>("control_period", 0.1);
  const std::string path_topic = this->declare_parameter<std::string>("path_topic", "/path");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  const std::string cmd_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  control_.configure(lookahead_distance, goal_tolerance, linear_speed,
                     max_angular_speed, turn_in_place_angle);

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(control_period), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.setPath(*msg);
  if (msg->poses.empty()) {
    RCLCPP_INFO(this->get_logger(), "Path cleared, stopping");
  } else {
    RCLCPP_INFO(this->get_logger(), "Following new path with %zu poses", msg->poses.size());
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
}

void ControlNode::timerCallback() {
  // Stay silent until odometry arrives: that proves the sim and its ROS<->GZ
  // bridge are up, and the bridge can crash if it sees a /cmd_vel during boot.
  if (!odom_) return;

  // From then on always publish, so a cleared path stops the robot rather
  // than leaving the last command running.
  geometry_msgs::msg::Twist cmd;
  if (control_.hasPath()) {
    cmd = control_.computeVelocity(*odom_);
  }
  cmd_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
