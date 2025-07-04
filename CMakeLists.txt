cmake_minimum_required(VERSION 3.8)
project(pcl_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(laser_geometry REQUIRED)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Export dependencies for ROSIDL
ament_export_dependencies(rosidl_default_runtime)


# ライブラリの定義とリンク
add_library(pcl_object_detection_lib
  # src/line_detection.cpp
  src/object_detection_floor.cpp
  src/object_detection_shelf.cpp
  src/object_detection_table.cpp
  src/placeable_detection.cpp
  src/point_cloud_processor.cpp 
)

target_link_libraries(pcl_object_detection_lib
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcl_object_detection_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  sobits_interfaces
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  PCL
)

# 実行可能ファイルの定義とリンク
add_executable(pcl_object_detection src/pcl_object_detection.cpp)
add_executable(pcl_line_detection src/line_detection.cpp)
target_link_libraries(pcl_object_detection
  pcl_object_detection_lib
)
target_link_libraries(pcl_line_detection
  pcl_object_detection_lib
)
ament_target_dependencies(pcl_object_detection
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  sobits_interfaces
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  PCL
)
ament_target_dependencies(pcl_line_detection
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  sobits_interfaces
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  PCL
  std_msgs
  std_srvs
)

# Install targets
install(TARGETS
  pcl_object_detection
  pcl_line_detection
  pcl_object_detection_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install param files
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
