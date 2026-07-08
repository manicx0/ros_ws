#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "bt_nodes/navigate_to_goal_action.cpp"
#include "bt_nodes/obstacle_check_condition.cpp"
#include "bt_nodes/recovery_rotate_action.cpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_executor_node");

  BT::BehaviorTreeFactory factory;

  // Register custom BT nodes, passing the ROS node as a constructor argument
  factory.registerNodeType<NavigateToGoal>("NavigateToGoal", node);
  factory.registerNodeType<ObstacleCheck>("ObstacleCheck", node);
  factory.registerNodeType<RecoveryRotate>("RecoveryRotate", node);

  // Locate BT XML files
  std::string bt_xml_dir;
  node->declare_parameter<std::string>("bt_xml_dir", "");
  node->get_parameter("bt_xml_dir", bt_xml_dir);

  if (bt_xml_dir.empty()) {
    bt_xml_dir = ament_index_cpp::get_package_share_directory("husky_bt") + "/bt_xml";
  }

  // Select which tree to run
  std::string tree_id;
  node->declare_parameter<std::string>("tree_id", "PatrolMission");
  node->get_parameter("tree_id", tree_id);

  std::string bt_xml_file = bt_xml_dir + "/" + (tree_id == "Navigate" ? "navigate_to_goal.xml" : "patrol_mission.xml");

  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", bt_xml_file.c_str());

  auto tree = factory.createTreeFromFile(bt_xml_file);

  // Enable logger
  BT::StdCoutLogger logger_cout(tree);

  rclcpp::WallRate rate(50);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    tree.tickWhileRunning();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
