cmake_minimum_required(VERSION 3.5)
project(route_follower)

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
  set(LIB_DIR \"\${CMAKE_INSTALL_PREFIX}/lib/route_follower\")

  # Create symlink for route_follower
  if(NOT EXISTS \"\${LIB_DIR}/route_follower\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E create_symlink
        \"\${BIN_DIR}/route_follower\"
        \"\${LIB_DIR}/route_follower\"
    )
    message(STATUS \"Created symlink: \${LIB_DIR}/route_follower -> \${BIN_DIR}/route_follower\")
  endif()

")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
