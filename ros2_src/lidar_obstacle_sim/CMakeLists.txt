cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_sim)

# ============================================================
# Compiler settings
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ============================================================
# Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gazebo_dev REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# ============================================================
# Build Gazebo Model Plugin
# ============================================================
add_library(moving_obstacle_plugin SHARED
  plugins/moving_obstacle_plugin.cpp
)

ament_target_dependencies(
  moving_obstacle_plugin
  rclcpp
  geometry_msgs
  gazebo_dev
  Eigen3
)

target_link_libraries(
  moving_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

# ============================================================
# Install Gazebo Model Plugin
# ============================================================
install(
  TARGETS moving_obstacle_plugin
  LIBRARY DESTINATION lib
)

# ============================================================
# Install Launch Files
# ============================================================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Worlds
# ============================================================
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Models
# ============================================================
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install Python Scripts
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

