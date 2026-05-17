#pragma once

// ROS 2 Lifecycle Core
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

// ROS 2 Messages & Tools
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL Core
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

// Project Utils
#include "pcl_object_detection/point_cloud_utility.hpp"

namespace pcl_object_detection {

/**
 * @class WashingMachineDetectionComponent
 * @brief Managed ROS 2 Lifecycle Component for detecting washing machine doors.
 */
class WashingMachineDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit WashingMachineDetectionComponent(const rclcpp::NodeOptions & options);

  // Lifecycle State Transitions
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /** @brief Main processing callback for incoming point clouds */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ROS 2 Communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  struct {
    std::string base_frame;
    std::string input_topic;
    std::string output_topic;
    
    // ROI Filtering
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

    // Plane Segmentation
    double plane_dist_threshold;

    // Circle Segmentation
    double circle_dist_threshold;
    double circle_radius_min;
    double circle_radius_max;

    // Temporal Smoothing
    double smoothing_alpha;

    // TF generation
    std::string opening_axis_side; // "left" or "right"
    double depth_shift;
    double rotation_offset;
  } params_;

  // Pre-allocated point clouds
  PointCloud::Ptr cloud_raw_;
  PointCloud::Ptr cloud_roi_;
  PointCloud::Ptr cloud_plane_;
  PointCloud::Ptr cloud_circle_;
  PointCloud::Ptr debug_cloud_;
  // Temporal filter state
  bool is_first_detection_ = true;
  Eigen::Vector3d last_center_;
  Eigen::Vector3d last_normal_;
};

}  // namespace pcl_object_detection
