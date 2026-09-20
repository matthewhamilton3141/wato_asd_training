#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 400);
  const int height = this->declare_parameter<int>("height", 400);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 2.0);
  const double min_range = this->declare_parameter<double>("min_range", 0.3);
  const std::string scan_topic = this->declare_parameter<std::string>("scan_topic", "/lidar");
  const std::string costmap_topic = this->declare_parameter<std::string>("costmap_topic", "/costmap");

  costmap_.configure(resolution, width, height, inflation_radius, min_range);

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::scanCallback, this, std::placeholders::_1));
}

void CostmapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.update(*msg);
  costmap_pub_->publish(costmap_.grid());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
