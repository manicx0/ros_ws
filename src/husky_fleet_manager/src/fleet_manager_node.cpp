#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"
#include "husky_msgs/msg/robot_state.hpp"
#include "husky_msgs/msg/goal_event.hpp"
#include "husky_msgs/msg/fleet_state.hpp"
#include "husky_msgs/msg/fleet_goal.hpp"
#include "husky_msgs/msg/fleet_result.hpp"
#include "husky_msgs/srv/set_robot_state.hpp"
#include "husky_msgs/srv/fleet_set_state.hpp"
#include "husky_msgs/action/navigate_to.hpp"
#include "husky_msgs/action/fleet_navigate.hpp"

struct RobotClient {
  std::string namespace_;
  rclcpp::Client<husky_msgs::srv::SetRobotState>::SharedPtr set_state_client;
  rclcpp_action::Client<husky_msgs::action::NavigateTo>::SharedPtr navigate_client;
  rclcpp::Subscription<husky_msgs::msg::RobotState>::SharedPtr state_sub;
  husky_msgs::msg::RobotState latest_state;
  bool state_received = false;
};

class FleetManagerNode : public rclcpp::Node {
public:
  using NavigateTo = husky_msgs::action::NavigateTo;
  using FleetNavigate = husky_msgs::action::FleetNavigate;
  using GoalHandleFleetNavigate = rclcpp_action::ServerGoalHandle<FleetNavigate>;

  FleetManagerNode() : Node("fleet_manager_node") {
    std::string config_file;
    declare_parameter<std::string>("fleet_config", "");
    get_parameter("fleet_config", config_file);

    if (config_file.empty()) {
      RCLCPP_ERROR(get_logger(), "fleet_config parameter not set");
      rclcpp::shutdown();
      return;
    }

    loadFleetConfig(config_file);

    fleet_state_pub_ = create_publisher<husky_msgs::msg::FleetState>("/fleet/robot_states", 10);
    goal_event_sub_ = create_subscription<husky_msgs::msg::GoalEvent>(
      "/fleet/goal_events", 10,
      std::bind(&FleetManagerNode::goalEventCallback, this, std::placeholders::_1));

    fleet_navigate_server_ = rclcpp_action::create_server<FleetNavigate>(
      this,
      "/fleet/fleet_navigate",
      std::bind(&FleetManagerNode::handleFleetGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FleetManagerNode::handleFleetCancel, this, std::placeholders::_1),
      std::bind(&FleetManagerNode::handleFleetAccepted, this, std::placeholders::_1));

    fleet_set_state_server_ = create_service<husky_msgs::srv::FleetSetState>(
      "/fleet/set_fleet_state",
      std::bind(&FleetManagerNode::fleetSetStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FleetManagerNode::publishFleetState, this));

    RCLCPP_INFO(get_logger(), "Fleet manager initialized with %zu robots", robots_.size());
  }

private:
  void loadFleetConfig(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    if (!config["fleet_manager"] || !config["fleet_manager"]["robots"]) {
      RCLCPP_ERROR(get_logger(), "Invalid fleet config format");
      return;
    }

    for (const auto& robot : config["fleet_manager"]["robots"]) {
      std::string ns = robot["namespace"].as<std::string>();
      RobotClient client;
      client.namespace_ = ns;

      client.set_state_client = create_client<husky_msgs::srv::SetRobotState>(
        "/" + ns + "/set_robot_state");
      client.navigate_client = rclcpp_action::create_client<NavigateTo>(
        this, "/" + ns + "/navigate_to");
      client.state_sub = create_subscription<husky_msgs::msg::RobotState>(
        "/" + ns + "/robot_state", 10,
        [this, ns](const husky_msgs::msg::RobotState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (robots_.count(ns)) {
            robots_[ns].latest_state = *msg;
            robots_[ns].state_received = true;
          }
        });

      robots_[ns] = std::move(client);
      RCLCPP_INFO(get_logger(), "Added robot: %s", ns.c_str());
    }
  }

  void goalEventCallback(const husky_msgs::msg::GoalEvent::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Goal event from %s: %s", msg->robot_id.c_str(), msg->type.c_str());
  }

  rclcpp_action::GoalResponse handleFleetGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FleetNavigate::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleFleetCancel(
    const std::shared_ptr<GoalHandleFleetNavigate>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleFleetAccepted(const std::shared_ptr<GoalHandleFleetNavigate> handle) {
    auto goal = handle->get_goal();
    auto results = std::make_shared<FleetNavigate::Result>();
    auto feedbacks = std::make_shared<FleetNavigate::Feedback>();

    std::map<std::string, bool> robot_completed;
    std::map<std::string, bool> robot_success;
    std::map<std::string, std::string> robot_message;

    for (const auto& fleet_goal : goal->goals) {
      const std::string& robot_id = fleet_goal.robot_id;
      if (!robots_.count(robot_id)) {
        RCLCPP_WARN(get_logger(), "Unknown robot: %s", robot_id.c_str());
        continue;
      }

      auto& client = robots_[robot_id];
      if (!client.navigate_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_WARN(get_logger(), "Action server not available for %s", robot_id.c_str());
        continue;
      }

      auto nav_goal = NavigateTo::Goal();
      nav_goal.target_pose = fleet_goal.target_pose;

      auto goal_handle_future = client.navigate_client->async_send_goal(nav_goal);

      auto result_future = client.navigate_client->async_get_result(goal_handle_future.get());
      auto result = result_future.get();

      husky_msgs::msg::FleetResult fleet_result;
      fleet_result.robot_id = robot_id;
      fleet_result.success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      fleet_result.message = fleet_result.success ? "Goal reached" : "Navigation failed";

      results->results.push_back(fleet_result);
    }

    handle->succeed(results);
  }

  void fleetSetStateCallback(
    const std::shared_ptr<husky_msgs::srv::FleetSetState::Request> request,
    std::shared_ptr<husky_msgs::srv::FleetSetState::Response> response) {
    for (const auto& robot_id : request->robot_ids) {
      if (!robots_.count(robot_id)) {
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Unknown robot";
        response->results.push_back(result);
        continue;
      }

      auto& client = robots_[robot_id];
      if (!client.set_state_client->wait_for_service(std::chrono::seconds(5))) {
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Service not available";
        response->results.push_back(result);
        continue;
      }

      auto srv_request = std::make_shared<husky_msgs::srv::SetRobotState::Request>();
      srv_request->waiting = request->waiting;
      srv_request->avoidance_enabled = request->avoidance_enabled;

      auto future = client.set_state_client->async_send_request(srv_request);
      auto srv_response = future.get();

      husky_msgs::msg::FleetResult result;
      result.robot_id = robot_id;
      result.success = srv_response->success;
      result.message = srv_response->message;
      response->results.push_back(result);
    }
  }

  void publishFleetState() {
    husky_msgs::msg::FleetState fleet_state;
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (auto& [ns, client] : robots_) {
      if (client.state_received) {
        fleet_state.robot_ids.push_back(ns);
        fleet_state.states.push_back(client.latest_state);
      }
    }

    fleet_state_pub_->publish(fleet_state);
  }

  std::map<std::string, RobotClient> robots_;
  std::mutex state_mutex_;

  rclcpp::Publisher<husky_msgs::msg::FleetState>::SharedPtr fleet_state_pub_;
  rclcpp::Subscription<husky_msgs::msg::GoalEvent>::SharedPtr goal_event_sub_;
  rclcpp_action::Server<FleetNavigate>::SharedPtr fleet_navigate_server_;
  rclcpp::Service<husky_msgs::srv::FleetSetState>::SharedPtr fleet_set_state_server_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FleetManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
