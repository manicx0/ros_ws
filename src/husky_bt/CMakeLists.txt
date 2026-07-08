cmake_minimum_required(VERSION 3.8)
project(husky_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED) # This finds the package
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(mission_executor_node
  src/mission_executor_node.cpp
)

# Use the exported variable from find_package
# Usually ${behaviortree_cpp_LIBRARIES} or just the package name
target_link_libraries(mission_executor_node
  behaviortree_cpp::behaviortree_cpp
)

ament_target_dependencies(mission_executor_node
  rclcpp geometry_msgs sensor_msgs nav_msgs ament_index_cpp
)

install(TARGETS mission_executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})

ament_package()
