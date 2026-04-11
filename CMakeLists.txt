cmake_minimum_required(VERSION 3.8)
project(pcl_object_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Include Directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Define the Component Library
add_library(detection_components SHARED
  src/preprocessor_component.cpp
  src/table_detection_component.cpp
  src/floor_detection_component.cpp
  src/shelf_detection_component.cpp
  src/placeable_detection_component.cpp
  src/line_detection_component.cpp
  src/basket_detection_component.cpp
)

# Link PCL separately (as it is not a standard ROS package)
target_link_libraries(detection_components
  ${PCL_LIBRARIES}
)

# Link ROS Dependencies
ament_target_dependencies(detection_components
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  std_msgs
  std_srvs
  laser_geometry
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

# Register Components
rclcpp_components_register_nodes(detection_components
  "pcl_object_detection::PreProcessorComponent"
  "pcl_object_detection::TableDetectionComponent"
  "pcl_object_detection::FloorDetectionComponent"
  "pcl_object_detection::ShelfDetectionComponent"
  "pcl_object_detection::PlaceableDetectionComponent"
  "pcl_object_detection::LineDetectionComponent"
  "pcl_object_detection::BasketDetectionComponent"
)

# Install Rules
install(TARGETS
  detection_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install Launch and Config files
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for other packages
ament_export_include_directories(include)
ament_export_libraries(detection_components)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  vision_msgs
  PCL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
