#pragma once

// ROS 2 Lifecycle Core
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

// ROS 2 Messages & Tools
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

// PCL Core
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

// Project Utils
#include "pcl_object_detection/point_cloud_utility.hpp"

namespace pcl_object_detection {

/**
 * @class PlaceableDetectionComponent
 * @brief Managed ROS 2 Lifecycle Component for finding an empty placement spot.
 * 
 * Extracts the table surface, maps the objects on it as obstacles, 
 * and performs a KD-Tree-accelerated grid search to find the safest 
 * (furthest from obstacles) empty spot to place an object.
 */
class PlaceableDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit PlaceableDetectionComponent(const rclcpp::NodeOptions & options);

  // Lifecycle State Transitions
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ROS 2 Communication
  // Dedicated callback group so the executor reliably services the
  // subscription created during a lifecycle transition (see .cpp).
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_cloud_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>> pub_detections_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // PCL Processing Objects
  PointCloud::Ptr cloud_filtered_;
  PointCloud::Ptr cloud_table_zone_;
  PointCloud::Ptr cloud_plane_;
  PointCloud::Ptr cloud_obstacles_;
  PointCloud::Ptr cloud_placeable_;  // grid spots that pass the clearance gate (debug view)

  pcl::SACSegmentation<PointT> seg_;
  pcl::ExtractIndices<PointT> extract_;
  pcl::search::KdTree<PointT>::Ptr tree_;

  // Parameters
  struct {
    std::string base_frame;
    
    // Area to look for
    double place_x_min, place_x_max;
    double place_y_min, place_y_max;
    double place_z_min, place_z_max;
    
    // Placement Logic
    double search_interval;     // Density of the grid (meters)
    double obstacle_tolerance;  // Min distance from any object (meters)
    double edge_margin;         // Safety distance from table edge (meters)

    // Plane segmentation parameters
    double plane_dist_threshold;
    int ransac_max_iterations;
  } params_;

  // QoS reliability settings
  std::string cloud_reliability_;
  std::string detections_pub_reliability_;
  std::string debug_pub_reliability_;

  // Pre-allocated message container
  vision_msgs::msg::Detection3DArray detection_msg_;
};

}  // namespace pcl_object_detection
