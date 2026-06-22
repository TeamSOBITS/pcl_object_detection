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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

// PCL Features
#include <pcl/features/normal_3d.h>

// Project Utils
#include "pcl_object_detection/point_cloud_utility.hpp"

#include <Eigen/Core>

namespace pcl_object_detection {

/**
 * @class BasketDetectionComponent
 * @brief Managed ROS 2 Lifecycle Component for detecting baskets and rectangular boxes.
 * 
 * Identifies the orientation and dimensions of rectangular containers and
 * locates graspable handles at the extremities of the longest axis.
 */
class BasketDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit BasketDetectionComponent(const rclcpp::NodeOptions & options);

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
  // Dedicated callback group so the executor reliably services the
  // subscription created during a lifecycle transition (see .cpp).
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filtered_cloud_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>> pub_detections_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // PCL Processing Objects
  PointCloud::Ptr cloud_filtered_;
  PointCloud::Ptr cloud_basket_candidates_;
  PointCloud::Ptr debug_cloud_;
  
  pcl::EuclideanClusterExtraction<PointT> ec_;
  pcl::search::KdTree<PointT>::Ptr tree_;

  // Parameters
  struct {
    std::string base_frame;

    // Lateral/forward ROI (base_frame) applied before clustering, to stop the
    // basket cluster merging with the floor, the washing machine and walls.
    double roi_x_min, roi_x_max;
    double roi_y_min, roi_y_max;

    // Detection zone (height range from floor)
    double detection_height_min;
    double detection_height_max;
  
    // Clustering parameters
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;

    // Basket dimension constraints
    double basket_width_min, basket_width_max;
    double basket_depth_min, basket_depth_max;
    double basket_height_min, basket_height_max;

    // Handle detection parameters
    double handle_search_radius; // How far from the edge to search for the highest point

    // Cloth detection parameters
    bool cloth_detection_enabled;
    double cloth_inner_margin;     // Padding from walls to define 'inside' the basket
    double cloth_top_band;         // Take the centroid of points within this depth of
                                   // the peak (not the single highest point, which
                                   // jitters badly on a deformable pile)
    double cloth_smoothing_alpha;  // EMA weight for the cloth pinch point

    // Frame/id prefix for emitted detection + TF frames.
    std::string detection_id_prefix;
  } params_;

  // Hot-reload handle for tunable parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // EMA state for the cloth pinch point
  Eigen::Vector3d smoothed_cloth_{0, 0, 0};
  bool smoothed_cloth_initialized_{false};

  // QoS reliability settings
  std::string cloud_reliability_;
  std::string detections_pub_reliability_;
  std::string debug_pub_reliability_;

  // Pre-allocated message container
  vision_msgs::msg::Detection3DArray detection_msg_;
};

}  // namespace pcl_object_detection
