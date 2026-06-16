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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

// Project Utils
#include "pcl_object_detection/point_cloud_utility.hpp"


namespace pcl_object_detection {

/**
 * @class TableDetectionComponent
 * @brief Managed ROS 2 Lifecycle Component for detecting objects on horizontal surfaces.
 * 
 * Subscribes to the filtered point cloud only when activated. Computes bounding
 * boxes using PCA to provide highly accurate 3D detections with orientation.
 */
class TableDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit TableDetectionComponent(const rclcpp::NodeOptions & options);

  // Lifecycle State Transitions
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /** @brief Main processing callback for incoming filtered point clouds */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ROS 2 Communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_cloud_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>> pub_detections_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // PCL Processing Objects
  PointCloud::Ptr cloud_filtered_;
  PointCloud::Ptr cloud_objects_;
  PointCloud::Ptr cloud_table_zone_;
  
  pcl::SACSegmentation<PointT> seg_;
  pcl::EuclideanClusterExtraction<PointT> ec_;
  pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::ExtractIndices<PointT> extract_;

  // Parameters
  struct {
    std::string base_frame;
  
    // Table physical bounds
    double table_height_min;
    double table_height_max;
  
    // Plane segmentation parameters
    double plane_dist_threshold;

    // Clustering parameters
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    int ransac_max_iterations;

    // Object size constraints
    double obj_x_min, obj_x_max;
    double obj_y_min, obj_y_max;
    double obj_z_min, obj_z_max;
  } params_;

  // QoS reliability settings
  std::string cloud_reliability_;
  std::string detections_pub_reliability_;
  std::string debug_pub_reliability_;

  // Pre-allocated message container
  vision_msgs::msg::Detection3DArray detection_msg_;
};

}  // namespace pcl_object_detection
