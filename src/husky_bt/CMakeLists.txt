find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(robot_localization REQUIRED)

ament_target_dependencies(mission_executor_node
  rclcpp behaviortree_cpp geometry_msgs sensor_msgs)
target_link_libraries(mission_executor_node BT::behaviortree_cpp)
