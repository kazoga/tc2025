cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_sim)

# Python3 を使うパッケージなので C++ 標準は特に不要だが、
# 他の依存がある場合のために残しておく
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# ============================================================
# Build Gazebo Model Plugin
# ============================================================
add_library(moving_obstacle_plugin SHARED plugins/moving_obstacle_plugin.cpp)
ament_target_dependencies(
  moving_obstacle_plugin
  rclcpp
  geometry_msgs
  gazebo_dev
)
target_link_libraries(moving_obstacle_plugin ${GAZEBO_LIBRARIES})
ament_export_targets(moving_obstacle_plugin HAS_LIBRARY_TARGET)
ament_export_libraries(moving_obstacle_plugin)
ament_export_dependencies(rclcpp geometry_msgs gazebo_dev)

# ============================================================
# Install Gazebo Model Plugin
# ============================================================
install(
  TARGETS moving_obstacle_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================
# Install Launch Files
# ============================================================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Worlds (6 files)
# ============================================================
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Models (全モデルフォルダごとコピー)
# ============================================================
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Python Nodes / Scripts
# ============================================================
install(
  PROGRAMS
    scripts/random_pylon_spawner.py
    scripts/fake_amcl_pose.py
    scripts/generate_waypoints.py
    scripts/road_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

