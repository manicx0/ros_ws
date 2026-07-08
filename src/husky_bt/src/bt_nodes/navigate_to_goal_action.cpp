// src/bt_nodes/navigate_to_goal_action.cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

class NavigateToGoal : public BT::StatefulActionNode {
public:
  NavigateToGoal(const std::string& name, const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
             BT::OutputPort<bool>("goal_reached") };
  }

  BT::NodeStatus onStart() override {
    getInput("goal", goal_);
    goal_pub_->publish(goal_);      // → /goal_waypoints
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (distanceToGoal() < 0.5) {
      setOutput("goal_reached", true);
      return BT::NodeStatus::SUCCESS;
    }
    if (obstacle_detected_) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { stopRobot(); }
};

// XML tree (bt_xml/navigate_to_goal.xml)
// <BehaviorTree ID="Navigate">
//   <Sequence>
//     <ObstacleCheck/>
//     <NavigateToGoal goal="{target_pose}"/>
//     <MissionComplete/>
//   </Sequence>
// </BehaviorTree>
