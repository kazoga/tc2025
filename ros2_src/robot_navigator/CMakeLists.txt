cmake_minimum_required(VERSION 3.5)
project(robot_navigator)

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
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/robot_navigator_node.py
  ${PROJECT_NAME}/robot_simulator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create symlinks for executables in lib/${PROJECT_NAME}
# This ensures compatibility with ExecutableInPackage substitution
install(CODE "
  set(BIN_DIR \"\${CMAKE_INSTALL_PREFIX}/bin\")
  set(LIB_DIR \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\")

  # Create symlink for robot_navigator
  if(NOT EXISTS \"\${LIB_DIR}/robot_navigator\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E create_symlink
        \"\${BIN_DIR}/robot_navigator\"
        \"\${LIB_DIR}/robot_navigator\"
    )
    message(STATUS \"Created symlink: \${LIB_DIR}/robot_navigator -> \${BIN_DIR}/robot_navigator\")
  endif()

  # Create symlink for robot_simulator
  if(NOT EXISTS \"\${LIB_DIR}/robot_simulator\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E create_symlink
        \"\${BIN_DIR}/robot_simulator\"
        \"\${LIB_DIR}/robot_simulator\"
    )
    message(STATUS \"Created symlink: \${LIB_DIR}/robot_simulator -> \${BIN_DIR}/robot_simulator\")
  endif()
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
