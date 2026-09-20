#include <chrono>
#include <cmath>

#include "map_memory_node.hpp"

using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 400);
  const int height = this->declare_parameter<int>("height", 400);
  const double origin_x = this->declare_parameter<double>("origin_x", -20.0);
  const double origin_y = this->declare_parameter<double>("origin_y", -20.0);
  const std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  const double update_period = this->declare_parameter<double>("update_period", 1.0);
  const std::string costmap_topic = this->declare_parameter<std::string>("costmap_topic", "/costmap");
  const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  const std::string map_topic = this->declare_parameter<std::string>("map_topic", "/map");

  map_memory_.configure(resolution, width, height, origin_x, origin_y, frame_id);

  // Latched so late subscribers (planner, Foxglove) get the current map immediately.
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local());

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period), std::bind(&MapMemoryNode::timerCallback, this));

  // Publish the (empty) map right away so downstream nodes have a frame to work in.
  map_memory_.map().header.stamp = this->now();
  map_pub_->publish(map_memory_.map());
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_buffer_.push_back(*msg);
  while (odom_buffer_.size() > 50) odom_buffer_.pop_front();

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  if (!have_last_update_pose_ || std::hypot(x - last_x_, y - last_y_) >= update_distance_) {
    should_update_ = true;
  }
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Pair the costmap with the pose closest to when its scan was taken, so a
  // turning robot doesn't smear far-away walls across the global map.
  auto odom = odomAt(rclcpp::Time(msg->header.stamp));
  if (!odom) return;
  latest_costmap_ = msg;
  latest_costmap_odom_ = odom;
}

std::optional<nav_msgs::msg::Odometry> MapMemoryNode::odomAt(const rclcpp::Time& stamp) const {
  std::optional<nav_msgs::msg::Odometry> best;
  double best_dt = 1e9;
  for (const auto& o : odom_buffer_) {
    const double dt = std::fabs((rclcpp::Time(o.header.stamp) - stamp).seconds());
    if (dt < best_dt) { best_dt = dt; best = o; }
  }
  return best;
}

void MapMemoryNode::timerCallback() {
  if (!should_update_ || !latest_costmap_ || !latest_costmap_odom_) return;

  const auto& pose = latest_costmap_odom_->pose.pose;
  const auto& q = pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  map_memory_.integrate(*latest_costmap_, pose.position.x, pose.position.y, yaw);

  auto& map = map_memory_.map();
  map.header.stamp = latest_costmap_->header.stamp;
  map.info.map_load_time = map.header.stamp;
  map_pub_->publish(map);

  last_x_ = pose.position.x;
  last_y_ = pose.position.y;
  have_last_update_pose_ = true;
  should_update_ = false;

  RCLCPP_INFO(this->get_logger(), "Map updated from pose (%.2f, %.2f, yaw %.2f)",
              last_x_, last_y_, yaw);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
