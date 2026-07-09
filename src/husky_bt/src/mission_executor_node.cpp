#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "husky_msgs/msg/robot_state.hpp"
#include "husky_msgs/srv/set_robot_state.hpp"

#include "bt_nodes/navigate_to_goal_action.cpp"
#include "bt_nodes/obstacle_check_condition.cpp"
#include "bt_nodes/recovery_rotate_action.cpp"
#include "bt_nodes/emergency_stop_condition.cpp"
#include "bt_nodes/waiting_condition.cpp"
#include "bt_nodes/idle_monitor.cpp"
#include "bt_nodes/avoidance_condition.cpp"
#include "bt_nodes/stop_and_wait.cpp"

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor_node") {
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      std::bind(&MissionExecutorNode::emergencyStopCallback, this, std::placeholders::_1));

    state_pub_ = create_publisher<husky_msgs::msg::RobotState>("robot_state", 10);

    set_state_srv_ = create_service<husky_msgs::srv::SetRobotState>(
      "set_robot_state",
      std::bind(&MissionExecutorNode::setStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MissionExecutorNode::publishState, this));

    emergency_stop_ = false;
    waiting_ = false;
    idle_ = true;
    avoidance_enabled_ = true;
    mission_active_ = false;
  }

  void setBlackboard(BT::Blackboard::Ptr blackboard) {
    blackboard_ = blackboard;
    blackboard_->set<bool>("emergency_stop", emergency_stop_);
    blackboard_->set<bool>("waiting", waiting_);
    blackboard_->set<bool>("idle", idle_);
    blackboard_->set<bool>("avoidance_enabled", avoidance_enabled_);
    blackboard_->set<bool>("mission_active", mission_active_);
    blackboard_->set<bool>("has_goal", false);
  }

private:
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_ = msg->data;
    if (blackboard_) {
      blackboard_->set<bool>("emergency_stop", emergency_stop_);
    }
  }

  void setStateCallback(
    const std::shared_ptr<husky_msgs::srv::SetRobotState::Request> request,
    std::shared_ptr<husky_msgs::srv::SetRobotState::Response> response) {
    waiting_ = request->waiting;
    avoidance_enabled_ = request->avoidance_enabled;

    if (blackboard_) {
      blackboard_->set<bool>("waiting", waiting_);
      blackboard_->set<bool>("avoidance_enabled", avoidance_enabled_);
    }

    response->success = true;
    response->message = "State updated";
  }

  void publishState() {
    if (blackboard_) {
      (void)blackboard_->get<bool>("idle", idle_);
      (void)blackboard_->get<bool>("mission_active", mission_active_);
    }

    husky_msgs::msg::RobotState state;
    state.emergency_stop = emergency_stop_;
    state.waiting = waiting_;
    state.idle = idle_;
    state.avoidance_enabled = avoidance_enabled_;
    state.mission_active = mission_active_;
    state_pub_->publish(state);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<husky_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Service<husky_msgs::srv::SetRobotState>::SharedPtr set_state_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  BT::Blackboard::Ptr blackboard_;

  bool emergency_stop_;
  bool waiting_;
  bool idle_;
  bool avoidance_enabled_;
  bool mission_active_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutorNode>();

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<NavigateToGoal>("NavigateToGoal", node);
  factory.registerNodeType<ObstacleCheck>("ObstacleCheck", node);
  factory.registerNodeType<RecoveryRotate>("RecoveryRotate", node);
  factory.registerNodeType<EmergencyStopCondition>("EmergencyStopCondition", node);
  factory.registerNodeType<WaitingCondition>("WaitingCondition", node);
  factory.registerNodeType<IdleMonitor>("IdleMonitor", node);
  factory.registerNodeType<AvoidanceEnabledCondition>("AvoidanceEnabledCondition", node);
  factory.registerNodeType<StopAndWait>("StopAndWait", node);

  std::string bt_xml_dir;
  node->declare_parameter<std::string>("bt_xml_dir", "");
  node->get_parameter("bt_xml_dir", bt_xml_dir);

  if (bt_xml_dir.empty()) {
    bt_xml_dir = ament_index_cpp::get_package_share_directory("husky_bt") + "/bt_xml";
  }

  std::string tree_id;
  node->declare_parameter<std::string>("tree_id", "PatrolMission");
  node->get_parameter("tree_id", tree_id);

  std::string bt_xml_file = bt_xml_dir + "/" + (tree_id == "Navigate" ? "navigate_to_goal.xml" : "patrol_mission.xml");

  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", bt_xml_file.c_str());

  auto tree = factory.createTreeFromFile(bt_xml_file);
  node->setBlackboard(tree.rootBlackboard());

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
