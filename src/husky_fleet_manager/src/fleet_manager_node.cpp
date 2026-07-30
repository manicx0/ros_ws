#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
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
  double home_x = 0.0;
  double home_y = 0.0;
  double home_yaw = 0.0;
};

struct ActiveGoal {
  std::string robot_id;
  std::shared_future<rclcpp_action::ClientGoalHandle<husky_msgs::action::NavigateTo>::SharedPtr> goal_handle_future;
  rclcpp_action::ClientGoalHandle<husky_msgs::action::NavigateTo>::SharedPtr goal_handle;
  std::shared_future<rclcpp_action::ClientGoalHandle<husky_msgs::action::NavigateTo>::WrappedResult> result_future;
  bool goal_accepted = false;
  bool result_received = false;
  bool success = false;
  std::string message;
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

    goal_monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FleetManagerNode::checkActiveGoals, this));

    RCLCPP_INFO(get_logger(), "Fleet manager initialized with %zu robots", robots_.size());
  }

  ~FleetManagerNode() {
    shutting_down_ = true;
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

      if (robot["home_pose"]) {
        client.home_x = robot["home_pose"]["x"].as<double>(0.0);
        client.home_y = robot["home_pose"]["y"].as<double>(0.0);
        client.home_yaw = robot["home_pose"]["yaw"].as<double>(0.0);
        RCLCPP_INFO(get_logger(), "Robot %s home pose: (%.2f, %.2f, %.2f)", 
                    ns.c_str(), client.home_x, client.home_y, client.home_yaw);
      }

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

    {
      std::lock_guard<std::mutex> lock(fleet_mutex_);
      active_fleet_goal_ = handle;
      fleet_results_ = std::make_shared<FleetNavigate::Result>();
      active_goals_.clear();
    }

    for (const auto& fleet_goal : goal->goals) {
      const std::string& robot_id = fleet_goal.robot_id;
      if (!robots_.count(robot_id)) {
        RCLCPP_WARN(get_logger(), "Unknown robot: %s", robot_id.c_str());
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Unknown robot";
        std::lock_guard<std::mutex> lock(fleet_mutex_);
        fleet_results_->results.push_back(result);
        continue;
      }

      auto& client = robots_[robot_id];
      if (!client.navigate_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_WARN(get_logger(), "Action server not available for %s", robot_id.c_str());
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Action server not available";
        std::lock_guard<std::mutex> lock(fleet_mutex_);
        fleet_results_->results.push_back(result);
        continue;
      }

      auto nav_goal = NavigateTo::Goal();
      nav_goal.target_pose = fleet_goal.target_pose;

      auto active_goal = std::make_shared<ActiveGoal>();
      active_goal->robot_id = robot_id;
      active_goal->goal_handle_future = client.navigate_client->async_send_goal(nav_goal);

      {
        std::lock_guard<std::mutex> lock(fleet_mutex_);
        active_goals_[robot_id] = active_goal;
      }
    }

    std::shared_ptr<GoalHandleFleetNavigate> handle_to_abort;
    std::shared_ptr<FleetNavigate::Result> results_to_abort;
    {
      std::lock_guard<std::mutex> lock(fleet_mutex_);
      if (active_goals_.empty()) {
        handle_to_abort = handle;
        results_to_abort = fleet_results_;
        active_fleet_goal_.reset();
      }
    }
    if (handle_to_abort) {
      handle_to_abort->abort(results_to_abort);
    }
  }

  void checkActiveGoals() {
    if (shutting_down_) {
      return;
    }

    std::shared_ptr<GoalHandleFleetNavigate> handle;
    std::shared_ptr<FleetNavigate::Result> results;

    {
      std::lock_guard<std::mutex> lock(fleet_mutex_);
      if (!active_fleet_goal_ || active_goals_.empty()) {
        return;
      }

      for (auto& [robot_id, active_goal] : active_goals_) {
        if (active_goal->result_received) {
          continue;
        }

        if (!active_goal->goal_accepted) {
          auto status = active_goal->goal_handle_future.wait_for(std::chrono::milliseconds(0));
          if (status == std::future_status::ready) {
            active_goal->goal_handle = active_goal->goal_handle_future.get();
            if (!active_goal->goal_handle) {
              active_goal->result_received = true;
              active_goal->success = false;
              active_goal->message = "Goal rejected";
              RCLCPP_WARN(get_logger(), "Goal rejected by %s", robot_id.c_str());
            } else {
              active_goal->goal_accepted = true;
              auto& robot_client = robots_[robot_id];
              active_goal->result_future = robot_client.navigate_client->async_get_result(active_goal->goal_handle);
            }
          }
        } else {
          auto status = active_goal->result_future.wait_for(std::chrono::milliseconds(0));
          if (status == std::future_status::ready) {
            auto result = active_goal->result_future.get();
            active_goal->result_received = true;
            active_goal->success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
            active_goal->message = active_goal->success ? "Goal reached" : "Navigation failed";
          }
        }
      }

      bool all_done = true;
      for (auto& [robot_id, active_goal] : active_goals_) {
        if (!active_goal->result_received) {
          all_done = false;
          break;
        }
      }

      if (!all_done) {
        return;
      }

      for (auto& [robot_id, active_goal] : active_goals_) {
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = active_goal->success;
        result.message = active_goal->message;
        fleet_results_->results.push_back(result);
      }

      handle = active_fleet_goal_;
      results = fleet_results_;
      active_fleet_goal_.reset();
      active_goals_.clear();
    }

    if (handle) {
      try {
        handle->succeed(results);
      } catch (const std::runtime_error& e) {
        RCLCPP_WARN(get_logger(), "Goal result publish skipped during shutdown: %s", e.what());
      }
    }
  }

  void fleetSetStateCallback(
    const std::shared_ptr<husky_msgs::srv::FleetSetState::Request> request,
    std::shared_ptr<husky_msgs::srv::FleetSetState::Response> response) {
    auto shared_response = response;
    auto pending_count = std::make_shared<size_t>(request->robot_ids.size());
    auto response_mutex = std::make_shared<std::mutex>();

    for (const auto& robot_id : request->robot_ids) {
      if (!robots_.count(robot_id)) {
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Unknown robot";
        std::lock_guard<std::mutex> lock(*response_mutex);
        response->results.push_back(result);
        (*pending_count)--;
        if (*pending_count == 0) {
          return;
        }
        continue;
      }

      auto& client = robots_[robot_id];
      if (!client.set_state_client->wait_for_service(std::chrono::seconds(5))) {
        husky_msgs::msg::FleetResult result;
        result.robot_id = robot_id;
        result.success = false;
        result.message = "Service not available";
        std::lock_guard<std::mutex> lock(*response_mutex);
        response->results.push_back(result);
        (*pending_count)--;
        if (*pending_count == 0) {
          return;
        }
        continue;
      }

      auto srv_request = std::make_shared<husky_msgs::srv::SetRobotState::Request>();
      srv_request->waiting = request->waiting;
      srv_request->avoidance_enabled = request->avoidance_enabled;

      client.set_state_client->async_send_request(
        srv_request,
        [this, robot_id, shared_response, response_mutex, pending_count](
          rclcpp::Client<husky_msgs::srv::SetRobotState>::SharedFuture future) {
          auto srv_response = future.get();
          husky_msgs::msg::FleetResult result;
          result.robot_id = robot_id;
          result.success = srv_response->success;
          result.message = srv_response->message;
          {
            std::lock_guard<std::mutex> lock(*response_mutex);
            shared_response->results.push_back(result);
            (*pending_count)--;
          }
        });
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
  std::mutex fleet_mutex_;

  rclcpp::Publisher<husky_msgs::msg::FleetState>::SharedPtr fleet_state_pub_;
  rclcpp::Subscription<husky_msgs::msg::GoalEvent>::SharedPtr goal_event_sub_;
  rclcpp_action::Server<FleetNavigate>::SharedPtr fleet_navigate_server_;
  rclcpp::Service<husky_msgs::srv::FleetSetState>::SharedPtr fleet_set_state_server_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr goal_monitor_timer_;

  std::shared_ptr<GoalHandleFleetNavigate> active_fleet_goal_;
  std::shared_ptr<FleetNavigate::Result> fleet_results_;
  std::map<std::string, std::shared_ptr<ActiveGoal>> active_goals_;
  bool shutting_down_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FleetManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
