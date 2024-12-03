cmake_minimum_required(VERSION 3.5)
project(simulations)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF file from robot_description to simulations package
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../robot_description/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
