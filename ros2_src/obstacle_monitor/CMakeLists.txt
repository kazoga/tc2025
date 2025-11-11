cmake_minimum_required(VERSION 3.5)
project(obstacle_monitor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install param files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install docs
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install tools
install(DIRECTORY
  tools
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install Python executables from the package directory
file(GLOB PYTHON_NODES "${PROJECT_NAME}/*_node.py")
install(PROGRAMS
  ${PYTHON_NODES}
  DESTINATION lib/${PROJECT_NAME}
)

# Create symlinks for executables in lib/${PROJECT_NAME}
# This ensures compatibility with ExecutableInPackage substitution
install(CODE "
  set(BIN_DIR \"\${CMAKE_INSTALL_PREFIX}/bin\")
  set(LIB_DIR \"\${CMAKE_INSTALL_PREFIX}/lib/obstacle_monitor\")

  # Create symlink for obstacle_monitor
  if(NOT EXISTS \"\${LIB_DIR}/obstacle_monitor\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E create_symlink
        \"\${BIN_DIR}/obstacle_monitor\"
        \"\${LIB_DIR}/obstacle_monitor\"
    )
    message(STATUS \"Created symlink: \${LIB_DIR}/obstacle_monitor -> \${BIN_DIR}/obstacle_monitor\")
  endif()

  # Create symlink for laser_scan_simulator
  if(NOT EXISTS \"\${LIB_DIR}/laser_scan_simulator\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E create_symlink
        \"\${BIN_DIR}/laser_scan_simulator\"
        \"\${LIB_DIR}/laser_scan_simulator\"
    )
    message(STATUS \"Created symlink: \${LIB_DIR}/laser_scan_simulator -> \${BIN_DIR}/laser_scan_simulator\")
  endif()

")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
