cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Build
################################################################################
# Torque Control Mode
################################################################################
add_executable(read_write_current_node src/read_write_current_node.cpp)
ament_target_dependencies(read_write_current_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
add_executable(pub_current_node src/pub_current_node.cpp)
ament_target_dependencies(pub_current_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

################################################################################
# Velocity Control Mode
################################################################################
add_executable(read_write_velocity_node src/read_write_velocity_node.cpp)
ament_target_dependencies(read_write_velocity_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
  geometry_msgs
)
add_executable(pub_velocity_node src/pub_velocity_node.cpp)
ament_target_dependencies(pub_velocity_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

################################################################################
# Position Control Mode
################################################################################
add_executable(read_write_position_node src/read_write_position_node.cpp)
ament_target_dependencies(read_write_position_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
add_executable(pub_position_node src/pub_position_node.cpp)
ament_target_dependencies(pub_position_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

################################################################################
# Current-based Position Control Mode
################################################################################
add_executable(read_write_current_pos_v1_node src/read_write_current_pos_v1_node.cpp)
ament_target_dependencies(read_write_current_pos_v1_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
add_executable(pub_current_pos_v1_node src/pub_current_pos_v1_node.cpp)
ament_target_dependencies(pub_current_pos_v1_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

################################################################################
# Current-based Position Control Mode ver2
################################################################################
add_executable(read_write_current_pos_v2_node src/read_write_current_pos_v2_node.cpp)
ament_target_dependencies(read_write_current_pos_v2_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
add_executable(pub_current_pos_v2_node src/pub_current_pos_v2_node.cpp)
ament_target_dependencies(pub_current_pos_v2_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

################################################################################
# Install
################################################################################
install(TARGETS
  read_write_current_node
  pub_current_node
  read_write_velocity_node
  pub_velocity_node
  read_write_position_node
  pub_position_node
  read_write_current_pos_v1_node
  pub_current_pos_v1_node
  read_write_current_pos_v2_node
  pub_current_pos_v2_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
