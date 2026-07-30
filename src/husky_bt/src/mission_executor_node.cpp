#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "husky_msgs/msg/robot_state.hpp"
#include "husky_msgs/msg/goal_event.hpp"
#include "husky_msgs/srv/set_robot_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "husky_msgs/action/navigate_to.hpp"

#include "bt_nodes/navigate_to_goal_action.cpp"
#include "bt_nodes/recovery_rotate_action.cpp"
#include "bt_nodes/recovery_reverse_action.cpp"
#include "bt_nodes/recovery_failed_action.cpp"
#include "bt_nodes/emergency_stop_condition.cpp"
#include "bt_nodes/waiting_condition.cpp"
#include "bt_nodes/idle_monitor.cpp"
#include "bt_nodes/stuck_check_condition.cpp"
#include "bt_nodes/gps_fix_check_condition.cpp"

class MissionExecutorNode : public rclcpp::Node {
public:
  using NavigateTo = husky_msgs::action::NavigateTo;
  using GoalHandleNavigateTo = rclcpp_action::ServerGoalHandle<NavigateTo>;

  static constexpr int EM_STOP_SOURCE_SOFTWARE = 0;
  static constexpr int EM_STOP_SOURCE_HARDWARE = 1;

  MissionExecutorNode() : Node("mission_executor_node") {
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      std::bind(&MissionExecutorNode::emergencyStopCallback, this, std::placeholders::_1));

    hardware_emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "platform/emergency_stop", 10,
      std::bind(&MissionExecutorNode::hardwareEmergencyStopCallback, this, std::placeholders::_1));

    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);

    state_pub_ = create_publisher<husky_msgs::msg::RobotState>("robot_state", 10);
    event_pub_ = create_publisher<husky_msgs::msg::GoalEvent>("/fleet/goal_events", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "platform/odom", 10,
      std::bind(&MissionExecutorNode::odomCallback, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "sensors/gps_0/fix", 10,
      std::bind(&MissionExecutorNode::gpsCallback, this, std::placeholders::_1));

    set_state_srv_ = create_service<husky_msgs::srv::SetRobotState>(
      "set_robot_state",
      std::bind(&MissionExecutorNode::setStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<NavigateTo>(
      this,
      "navigate_to",
      std::bind(&MissionExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MissionExecutorNode::handleCancel, this, std::placeholders::_1),
      std::bind(&MissionExecutorNode::handleAccepted, this, std::placeholders::_1));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MissionExecutorNode::onTimer, this));

    emergency_stop_ = false;
    emergency_stop_source_ = EM_STOP_SOURCE_SOFTWARE;
    waiting_ = false;
    idle_ = true;
    avoidance_enabled_ = true;
    mission_active_ = false;
    waiting_for_result_ = false;
    has_odom_ = false;
  }

  void initialize(BT::Tree&& tree) {
    tree_ = std::move(tree);
    blackboard_ = tree_.rootBlackboard();
    blackboard_->set<bool>("emergency_stop", emergency_stop_);
    blackboard_->set<bool>("waiting", waiting_);
    blackboard_->set<bool>("idle", idle_);
    blackboard_->set<bool>("avoidance_enabled", avoidance_enabled_);
    blackboard_->set<bool>("mission_active", mission_active_);
    blackboard_->set<bool>("has_goal", false);
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
  }

  void tick() {
    tree_.tickWhileRunning();
  }

private:
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_ = msg->data;
    if (emergency_stop_) {
      emergency_stop_source_ = EM_STOP_SOURCE_SOFTWARE;
    }
    if (blackboard_) {
      blackboard_->set<bool>("emergency_stop", emergency_stop_);
    }
  }

  void hardwareEmergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_ = msg->data;
    if (emergency_stop_) {
      emergency_stop_source_ = EM_STOP_SOURCE_HARDWARE;
    }
    if (blackboard_) {
      blackboard_->set<bool>("emergency_stop", emergency_stop_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_linear_vel_ = msg->twist.twist.linear.x;
    odom_angular_vel_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gps_latitude_ = msg->latitude;
    gps_longitude_ = msg->longitude;
    gps_fix_valid_ = (msg->status.status >= 0);
    has_gps_ = true;
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

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const NavigateTo::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigateTo> handle) {
    publishZeroVelocity();
    if (blackboard_) {
      blackboard_->set<bool>("mission_active", false);
    }
    tree_.haltTree();
    if (blackboard_) {
      blackboard_->set<bool>("has_goal", false);
    }

    auto result = std::make_shared<NavigateTo::Result>();
    result->success = false;
    result->message = "Cancelled";
    handle->abort(result);
    waiting_for_result_ = false;
    active_goal_.reset();

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigateTo> handle) {
    if (active_goal_ && waiting_for_result_) {
      publishZeroVelocity();
      if (blackboard_) {
        blackboard_->set<bool>("mission_active", false);
      }
      tree_.haltTree();

      auto old_result = std::make_shared<NavigateTo::Result>();
      old_result->success = false;
      old_result->message = "Preempted by new goal";
      active_goal_->abort(old_result);
      active_goal_.reset();
    }

    if (emergency_stop_ && emergency_stop_source_ == EM_STOP_SOURCE_SOFTWARE) {
      RCLCPP_INFO(get_logger(), "Auto-clearing software emergency_stop on new goal");
      emergency_stop_ = false;
      if (blackboard_) {
        blackboard_->set<bool>("emergency_stop", false);
      }
      std_msgs::msg::Bool cmd;
      cmd.data = false;
      emergency_stop_pub_->publish(cmd);
    }

    if (waiting_) {
      RCLCPP_INFO(get_logger(), "Auto-clearing waiting flag on new goal");
      waiting_ = false;
      if (blackboard_) {
        blackboard_->set<bool>("waiting", false);
      }
    }

    if (blackboard_) {
      blackboard_->set<geometry_msgs::msg::PoseStamped>("target_pose", handle->get_goal()->target_pose);
      blackboard_->set<bool>("has_goal", true);
      blackboard_->set<int>("recovery_attempts", 0);
    }

    active_goal_ = handle;
    waiting_for_result_ = true;
  }

  void onTimer() {
    publishState();
    checkActiveGoal();
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

    state.odom_valid = has_odom_;
    if (has_odom_) {
      state.position_x = odom_x_;
      state.position_y = odom_y_;
      state.position_yaw = odom_yaw_;
      state.linear_velocity = odom_linear_vel_;
      state.angular_velocity = odom_angular_vel_;
    }

    if (has_gps_) {
      state.gps_latitude = gps_latitude_;
      state.gps_longitude = gps_longitude_;
      state.gps_fix_valid = gps_fix_valid_;
    }

    state_pub_->publish(state);
  }

  void checkActiveGoal() {
    if (!waiting_for_result_ || !active_goal_ || !blackboard_) {
      return;
    }

    bool emergency_stop = false;
    (void)blackboard_->get<bool>("emergency_stop", emergency_stop);
    if (emergency_stop) {
      publishZeroVelocity();
      blackboard_->set<bool>("mission_active", false);
      tree_.haltTree();
      blackboard_->set<bool>("has_goal", false);

      auto result = std::make_shared<NavigateTo::Result>();
      result->success = false;
      result->message = "Emergency stop";
      active_goal_->abort(result);
      waiting_for_result_ = false;
      active_goal_.reset();
      return;
    }

    bool recovery_failed = false;
    (void)blackboard_->get<bool>("recovery_failed", recovery_failed);
    if (recovery_failed) {
      publishZeroVelocity();
      blackboard_->set<bool>("mission_active", false);
      blackboard_->set<bool>("has_goal", false);
      blackboard_->set<bool>("recovery_failed", false);

      auto result = std::make_shared<NavigateTo::Result>();
      result->success = false;
      result->message = "Stuck - recovery failed";
      active_goal_->abort(result);
      waiting_for_result_ = false;
      active_goal_.reset();
      return;
    }

    bool has_goal = true;
    (void)blackboard_->get<bool>("has_goal", has_goal);

    if (has_goal) {
      auto feedback = std::make_shared<NavigateTo::Feedback>();
      bool obstacle = false;
      (void)blackboard_->get<bool>("obstacle_detected", obstacle);

      if (obstacle) {
        feedback->status = "OBSTACLE_BLOCKED";
        feedback->description = "Obstacle detected, robot stopped";
        publishGoalEvent("OBSTACLE_BLOCKED");
      } else {
        feedback->status = "NAVIGATING";
        feedback->description = "Navigating to goal";
        publishGoalEvent("NAVIGATING");
      }
      active_goal_->publish_feedback(feedback);
      return;
    }

    bool goal_reached = false;
    (void)blackboard_->get<bool>("goal_reached", goal_reached);

    auto result = std::make_shared<NavigateTo::Result>();
    if (goal_reached) {
      result->success = true;
      result->message = "Goal reached";
      active_goal_->succeed(result);
    } else {
      result->success = false;
      result->message = "Navigation incomplete";
      active_goal_->abort(result);
    }

    waiting_for_result_ = false;
    active_goal_.reset();
  }

  void publishZeroVelocity() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishGoalEvent(const std::string& type) {
    husky_msgs::msg::GoalEvent event;
    event.type = type;
    event.robot_id = get_namespace();
    if (blackboard_) {
      geometry_msgs::msg::PoseStamped goal;
      if (blackboard_->get<geometry_msgs::msg::PoseStamped>("target_pose", goal)) {
        event.goal = goal;
      }
    }
    event_pub_->publish(event);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_emergency_stop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<husky_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<husky_msgs::msg::GoalEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Service<husky_msgs::srv::SetRobotState>::SharedPtr set_state_srv_;
  rclcpp_action::Server<NavigateTo>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;
  BT::Blackboard::Ptr blackboard_;

  bool emergency_stop_;
  int emergency_stop_source_;
  bool waiting_;
  bool idle_;
  bool avoidance_enabled_;
  bool mission_active_;
  bool waiting_for_result_;
  std::shared_ptr<GoalHandleNavigateTo> active_goal_;

  bool has_odom_;
  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  double odom_linear_vel_;
  double odom_angular_vel_;

  bool has_gps_{false};
  double gps_latitude_{0.0};
  double gps_longitude_{0.0};
  bool gps_fix_valid_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutorNode>();

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<NavigateToGoal>("NavigateToGoal", node);
  factory.registerNodeType<RecoveryRotate>("RecoveryRotate", node);
  factory.registerNodeType<RecoveryReverse>("RecoveryReverse", node);
  factory.registerNodeType<RecoveryFailed>("RecoveryFailed", node);
  factory.registerNodeType<EmergencyStopCondition>("EmergencyStopCondition", node);
  factory.registerNodeType<WaitingCondition>("WaitingCondition", node);
  factory.registerNodeType<IdleMonitor>("IdleMonitor", node);
  factory.registerNodeType<StuckCheck>("StuckCheck", node);
  factory.registerNodeType<GpsFixCheck>("GpsFixCheck", node);

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
  node->initialize(std::move(tree));

  rclcpp::WallRate rate(50);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->tick();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
