cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Install launch, worlds, models into share directory
install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(
  PROGRAMS
    scripts/spawn_random_pylons.py
    scripts/fake_amcl_pose.py
    scripts/generate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
