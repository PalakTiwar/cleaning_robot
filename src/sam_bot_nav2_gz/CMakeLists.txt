cmake_minimum_required(VERSION 3.5)
project(sam_bot_nav2_gz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# Add required packages here:
find_package(control_msgs REQUIRED) # Added: For control messages
find_package(ros_ign_gazebo REQUIRED) # Added: For Ignition Gazebo ROS 2 integration
find_package(rosidl_default_generators REQUIRED) # Re-added: Needed for custom message generation


# Generate custom message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectedObjects.msg"
)

# Install directories
# Removed 'scripts' from here as Python executables are handled by setup.py's entry_points
install(
  DIRECTORY src launch rviz config world models
  DESTINATION share/${PROJECT_NAME}
)

# Removed this block, as Python executables defined in setup.py's console_scripts
# are automatically handled by ament_cmake_python during the build process.
# install(PROGRAMS
#   scripts/reach_goal.py
#   scripts/follow_waypoints.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies to allow other packages to find your custom messages at runtime
ament_export_dependencies(rosidl_default_runtime)
#ament_python_install_package(sam_bot_nav2_gz_py)
ament_package()
