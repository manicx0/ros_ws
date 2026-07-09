cmake_minimum_required(VERSION 3.8)
project(husky_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/GoalEvent.msg"
  "msg/FleetState.msg"
  "msg/FleetGoal.msg"
  "msg/FleetResult.msg"
  "srv/SetRobotState.srv"
  "srv/FleetSetState.srv"
  "action/NavigateTo.action"
  "action/FleetNavigate.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
