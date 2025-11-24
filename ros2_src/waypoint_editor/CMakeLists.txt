cmake_minimum_required(VERSION 3.8)
project(waypoint_editor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SearchLabel.srv"
  "srv/UpdatePose.srv"
  "srv/UpdateFlags.srv"
  "srv/UpdateNode.srv"
  "srv/ShiftWaypoint.srv"
  "srv/RotateWaypoint.srv"
  "srv/SaveWaypoints.srv"
  "srv/ResetWaypoints.srv"
  "srv/DeleteWaypoint.srv"
  "srv/SelectInRviz.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  waypoint_editor/waypoint_editor_node.py
  waypoint_editor/waypoint_editor_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclpy
  std_msgs
  geometry_msgs
  visualization_msgs
  rosidl_default_runtime
)

ament_package()
