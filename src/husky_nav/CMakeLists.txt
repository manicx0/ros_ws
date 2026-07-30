cmake_minimum_required(VERSION 3.8)
project(husky_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# 1. Pure Pursuit Node
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs)

# 2. Obstacle Detector Node
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_detector_node rclcpp sensor_msgs geometry_msgs pcl_conversions)

# 3. VFH Planner Node (replaces path_planner + pure_pursuit)
add_executable(vfh_planner_node src/vfh_planner_node.cpp)
ament_target_dependencies(vfh_planner_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs husky_msgs tf2 tf2_geometry_msgs)

# 4. EKF GPS Wrapper Node
add_executable(ekf_gps_node src/ekf_gps_node.cpp)
ament_target_dependencies(ekf_gps_node rclcpp)

# 5. Stuck Detector Node
add_executable(stuck_detector_node src/stuck_detector_node.cpp)
ament_target_dependencies(stuck_detector_node rclcpp geometry_msgs nav_msgs std_msgs husky_msgs)

# 6. Topic Health Monitor Node
add_executable(topic_health_node src/topic_health_node.cpp)
ament_target_dependencies(topic_health_node rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs)

# Install targets
install(TARGETS
  pure_pursuit_node
  obstacle_detector_node
  vfh_planner_node
  ekf_gps_node
  stuck_detector_node
  topic_health_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
