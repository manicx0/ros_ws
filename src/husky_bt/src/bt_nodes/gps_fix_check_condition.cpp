#include <functional>
#include <behaviortree_cpp/condition_node.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsFixCheck : public BT::SimpleConditionNode {
public:
  GpsFixCheck(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SimpleConditionNode(name, std::bind(&GpsFixCheck::checkFix, this), config), node_(node) {
    start_time_ = node_->now();
    gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      "sensors/gps_0/fix", 10,
      std::bind(&GpsFixCheck::gpsCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_fix_time_ = node_->now();
    fix_status_ = msg->status.status;
    has_fix_ = (fix_status_ != -1);
    has_received_ = true;
  }

  BT::NodeStatus checkFix() {
    double uptime = (node_->now() - start_time_).seconds();

    if (has_received_) {
      RCLCPP_DEBUG(node_->get_logger(),
        "GpsFixCheck: fix_status=%d has_fix=%d age=%.1fs",
        fix_status_, has_fix_, (node_->now() - last_fix_time_).seconds());
    } else {
      RCLCPP_DEBUG(node_->get_logger(),
        "GpsFixCheck: no data received yet (uptime=%.1fs)", uptime);
    }

    if (uptime > 5.0) {
      if (!has_received_) {
        RCLCPP_WARN(node_->get_logger(),
          "No GPS data received after 5s — proceeding without GPS");
      } else if (!has_fix_) {
        RCLCPP_WARN(node_->get_logger(),
          "GPS has NO_FIX (status=%d) after 5s — proceeding without valid fix",
          fix_status_);
      } else if ((node_->now() - last_fix_time_).seconds() > 5.0) {
        RCLCPP_WARN(node_->get_logger(),
          "GPS fix stale for %.1fs — proceeding without GPS",
          (node_->now() - last_fix_time_).seconds());
      }
      config().blackboard->set<bool>("gps_fix_lost", false);
      return BT::NodeStatus::SUCCESS;
    }

    if (!has_received_) {
      config().blackboard->set<bool>("gps_fix_lost", true);
      return BT::NodeStatus::FAILURE;
    }

    auto elapsed = (node_->now() - last_fix_time_).seconds();
    if (elapsed > 5.0) {
      config().blackboard->set<bool>("gps_fix_lost", true);
      return BT::NodeStatus::FAILURE;
    }

    if (!has_fix_) {
      config().blackboard->set<bool>("gps_fix_lost", true);
      return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set<bool>("gps_fix_lost", false);
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Time start_time_;
  rclcpp::Time last_fix_time_;
  int fix_status_ = -1;
  bool has_fix_ = false;
  bool has_received_ = false;
};
