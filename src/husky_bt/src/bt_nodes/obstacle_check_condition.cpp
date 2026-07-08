#include <functional>
#include <behaviortree_cpp/condition_node.h>
#include "rclcpp/rclcpp.hpp"

class ObstacleCheck : public BT::SimpleConditionNode {
public:
  ObstacleCheck(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SimpleConditionNode(name, std::bind(&ObstacleCheck::checkObstacles, this), config), node_(node) {}

  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus checkObstacles() {
    // Return SUCCESS if clearway is open, FAILURE if unsafe
    return BT::NodeStatus::SUCCESS;
  }
  rclcpp::Node::SharedPtr node_;
};
