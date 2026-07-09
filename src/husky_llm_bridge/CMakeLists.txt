cmake_minimum_required(VERSION 3.8)
project(husky_llm_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(husky_llm_bridge)

install(PROGRAMS
  husky_llm_bridge/llm_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
