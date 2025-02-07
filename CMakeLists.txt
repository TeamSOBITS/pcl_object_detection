cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)

project(pcl_object_detection)

# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(yaml-cpp REQUIRED CONFIG)

# Export dependencies for ROSIDL
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}  # yaml-cpp のヘッダを含める
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# ライブラリの定義とリンク
add_library(pcl_object_detection_lib
  src/mode_manager_node.cpp
  src/line_detection.cpp
  src/object_detection_floor.cpp
  src/object_detection_shelf.cpp
  src/object_detection_table.cpp
  src/placeable_detection.cpp
  src/point_cloud_processor.cpp 
)

target_link_libraries(pcl_object_detection_lib
  ${PCL_LIBRARIES}
  ${yaml-cpp_LIBRARIES}  #  追加
  yaml-cpp
)
ament_target_dependencies(pcl_object_detection_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  sobits_interfaces
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  PCL
  yaml-cpp
  pluginlib  # ここで `pluginlib::pluginlib` を統合
)

# 実行可能ファイルの定義とリンク
add_executable(mode_manager_node src/mode_manager_node.cpp)
target_link_libraries(mode_manager_node
  pcl_object_detection_lib
  ${yaml-cpp_LIBRARIES}  #  追加
)
ament_target_dependencies(mode_manager_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  sobits_interfaces
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  PCL
  yaml-cpp
)

# Install targets
install(TARGETS
  mode_manager_node
  pcl_object_detection_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install launch files
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
