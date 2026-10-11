cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required ROS 2 and external dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ==========================================
# 1. Build the Perception Node (vision_node) - "The Eyes"
# ==========================================
add_executable(vision_node src/vision_node.cpp)

# Include the directory containing header (.hpp) files
target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies for computer vision and image processing
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# ==========================================
# 2. Build the Behavior Node (behavior_node) - "The Brain"
# ==========================================
add_executable(behavior_node src/behavior_node.cpp)

# Include the directory containing header (.hpp) files
target_include_directories(behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies required for behavior logic, velocity commands, and sensor data
ament_target_dependencies(behavior_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# ==========================================
# 3. Install Directives
# ==========================================
# Install the compiled executable binaries so ROS 2 can find them via 'ros2 run'
install(TARGETS
  vision_node
  behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files (e.g., YAML parameters)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()