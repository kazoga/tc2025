cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_sim)

# Python3 を使うパッケージなので C++ 標準は特に不要だが、
# 他の依存がある場合のために残しておく
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

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

