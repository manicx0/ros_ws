#include <functional>
#include <behaviortree_cpp/condition_node.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleCheck : public BT::SimpleConditionNode {
public:
  ObstacleCheck(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SimpleConditionNode(name, std::bind(&ObstacleCheck::checkObstacles, this), config), node_(node) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_2d", 10,
      std::bind(&ObstacleCheck::scanCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double min_range = msg->range_max;
    size_t obstacle_count = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float angle = msg->angle_min + i * msg->angle_increment;
      if (angle > -0.5 && angle < 0.5) {
        if (msg->ranges[i] < min_range && msg->ranges[i] > msg->range_min) {
          min_range = msg->ranges[i];
          if (msg->ranges[i] < 1.5) {
            obstacle_count++;
          }
        }
      }
    }

    obstacle_detected_ = (obstacle_count > 3);
  }

  BT::NodeStatus checkObstacles() {
    config().blackboard->set<bool>("obstacle_detected", obstacle_detected_);
    return obstacle_detected_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool obstacle_detected_ = false;
};
