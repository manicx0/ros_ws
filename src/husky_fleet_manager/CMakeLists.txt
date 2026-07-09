cmake_minimum_required(VERSION 3.8)
project(husky_fleet_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(fleet_manager_node src/fleet_manager_node.cpp)

ament_target_dependencies(fleet_manager_node
  rclcpp rclcpp_action husky_msgs geometry_msgs std_msgs
)

target_link_libraries(fleet_manager_node yaml-cpp)

install(TARGETS fleet_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
