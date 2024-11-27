cmake_minimum_required(VERSION 3.10)
project(trajectory)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create ROS2 node executable from source files
add_executable(trajectory_node src/trajectory_node.cpp)
target_include_directories(trajectory_node
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(trajectory_node rclcpp geometry_msgs nav_msgs)

# Copy executable to installation location
install(TARGETS
  trajectory_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
