cmake_minimum_required(VERSION 3.8)
project(tank_drive_controller_old)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tank_interface REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(tank_drive_node src/tank_drive_controller_old.cpp)
ament_target_dependencies(tank_drive_node
  rclcpp
  geometry_msgs
  tank_interface
)

# Install the executable
install(TARGETS
  tank_drive_node
  DESTINATION lib/${PROJECT_NAME})

# Install other files (e.g., launch and config)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
