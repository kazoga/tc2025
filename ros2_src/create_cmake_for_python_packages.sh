#!/bin/bash
# ROS2 Python packages に CMakeLists.txt を追加して、
# bin/ から lib/<package>/ へのシンボリックリンクを作成するスクリプト

PACKAGES=(
  "route_follower:route_follower"
  "obstacle_monitor:obstacle_monitor,laser_scan_simulator"
  "robot_console:robot_console"
)

for pkg_info in "${PACKAGES[@]}"; do
  IFS=':' read -r pkg_name executables <<< "$pkg_info"
  pkg_dir="/home/nkb/ros/tc2025/ros2_src/${pkg_name}"

  echo "Processing package: ${pkg_name}"

  # CMakeLists.txt を作成
  cat > "${pkg_dir}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.5)
project(${pkg_name})

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
ament_python_install_package(\${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/\${PROJECT_NAME}/
)

# Install param files
install(DIRECTORY
  params
  DESTINATION share/\${PROJECT_NAME}/
  OPTIONAL
)

# Install docs
install(DIRECTORY
  docs
  DESTINATION share/\${PROJECT_NAME}/
  OPTIONAL
)

# Install tools
install(DIRECTORY
  tools
  DESTINATION share/\${PROJECT_NAME}/
  OPTIONAL
)

# Install Python executables from the package directory
file(GLOB PYTHON_NODES "\${PROJECT_NAME}/*_node.py")
install(PROGRAMS
  \${PYTHON_NODES}
  DESTINATION lib/\${PROJECT_NAME}
)

# Create symlinks for executables in lib/\${PROJECT_NAME}
# This ensures compatibility with ExecutableInPackage substitution
install(CODE "
  set(BIN_DIR \\"\\\${CMAKE_INSTALL_PREFIX}/bin\\")
  set(LIB_DIR \\"\\\${CMAKE_INSTALL_PREFIX}/lib/${pkg_name}\\")

EOF

  # 各実行可能ファイルのシンボリックリンク作成コードを追加
  IFS=',' read -ra EXECS <<< "$executables"
  for exec in "${EXECS[@]}"; do
    cat >> "${pkg_dir}/CMakeLists.txt" <<EOF
  # Create symlink for ${exec}
  if(NOT EXISTS \\"\\\${LIB_DIR}/${exec}\\")
    execute_process(
      COMMAND \\\${CMAKE_COMMAND} -E create_symlink
        \\"\\\${BIN_DIR}/${exec}\\"
        \\"\\\${LIB_DIR}/${exec}\\"
    )
    message(STATUS \\"Created symlink: \\\${LIB_DIR}/${exec} -> \\\${BIN_DIR}/${exec}\\")
  endif()

EOF
  done

  cat >> "${pkg_dir}/CMakeLists.txt" <<EOF
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF

  echo "  Created CMakeLists.txt for ${pkg_name}"

  # package.xml を更新
  sed -i 's/<buildtool_depend>ament_cmake<\/buildtool_depend>/<buildtool_depend>ament_cmake<\/buildtool_depend>\n  <buildtool_depend>ament_cmake_python<\/buildtool_depend>/' "${pkg_dir}/package.xml"
  sed -i 's/<build_type>ament_python<\/build_type>/<build_type>ament_cmake<\/build_type>/' "${pkg_dir}/package.xml"
  sed -i 's/<exec_depend>rclpy<\/exec_depend>/<depend>rclpy<\/depend>/' "${pkg_dir}/package.xml"

  echo "  Updated package.xml for ${pkg_name}"
done

echo "All packages processed successfully!"
