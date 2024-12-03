cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Add any other dependencies here, e.g.,
# find_package(rclcpp REQUIRED)

# Install URDF/XACRO files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)



# Install other directories, such as config, if applicable
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
