#include "rclcpp/rclcpp.hpp"

class EkgGpsNodeWrapper : public rclcpp::Node {
public:
  EkgGpsNodeWrapper() : Node("ekf_gps_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Node Wrapper for EKF robot_localization ecosystem setup.");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkgGpsNodeWrapper>());
  rclcpp::shutdown();
  return 0;
}
