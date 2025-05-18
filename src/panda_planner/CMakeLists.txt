cmake_minimum_required(VERSION 3.10)
project(panda_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(planner_node src/planner_node.cpp)

set_target_properties(planner_node PROPERTIES OUTPUT_NAME "planner_node")

ament_target_dependencies(planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_vision_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


