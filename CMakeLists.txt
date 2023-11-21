cmake_minimum_required(VERSION 3.10.2)
project(pcl_object_detection)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  laser_geometry
  message_generation
  pluginlib
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sobit_common_msg
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LineInfo.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectionModeCtr.srv
  LineDetectionService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   std_msgs   visualization_msgs
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/ObjectDetectionParameter.cfg
  config/dynamic_reconfigure/PCLParameter.cfg
  config/dynamic_reconfigure/LineDetectionParameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  laser_geometry
  pluginlib
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sobit_common_msg
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_save_pcd src/sub/save_pcd.cpp)
target_link_libraries(${PROJECT_NAME}_save_pcd  ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_save_pcd ${PROJECT_NAME}_generate_messages_cpp)

add_executable(${PROJECT_NAME}_pointcloud_publisher src/sub/pointcloud_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_pointcloud_publisher  ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_pointcloud_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(${PROJECT_NAME}_scan_publisher src/sub/scan_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_scan_publisher  ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_scan_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_library(${PROJECT_NAME} src/detection_manager_nodelet.cpp
                            src/object_detection_table_nodelet.cpp
                            src/object_detection_floor_nodelet.cpp
                            src/object_detection_shelf_nodelet.cpp
                            src/placeable_detection_nodelet.cpp
                            src/point_cloud_processor.cpp
                            src/line_detection_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})