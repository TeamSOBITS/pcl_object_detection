#pragma once

// ROS 2 Lifecycle Core
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

// ROS 2 Messages & Tools
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>

// PCL Core
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Project Utils
#include "pcl_object_detection/point_cloud_utility.hpp"

namespace pcl_object_detection {

/**
 * @class LineDetectionComponent
 * @brief Managed ROS 2 Lifecycle Component for detecting straight lines from 2D LiDAR.
 * 
 * Subscribes to LaserScan only when active, projects it to 3D, and uses RANSAC
 * to find the dominant line (e.g., a wall). Outputs the pose of the line.
 */
class LineDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit LineDetectionComponent(const rclcpp::NodeOptions & options);

  // Lifecycle State Transitions
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /** @brief Callback for handling incoming LaserScan messages */
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // ROS 2 Communication
  // Dedicated callback group so the executor reliably services the
  // subscription created during a lifecycle transition (see .cpp).
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_line_cloud_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> pub_line_pose_;
  
  // PCL Processing Objects
  laser_geometry::LaserProjection projector_;
  PointCloud::Ptr cloud_raw_;
  PointCloud::Ptr cloud_line_;
  
  pcl::SACSegmentation<PointT> seg_;
  pcl::ExtractIndices<PointT> extract_;

  // Parameters
  struct {
    std::string base_frame;
  
    // RANSAC Parameters
    double passthrough_min;
    double passthrough_max;
    std::string passthrough_axis;

    // Line Model Parameters
    double distance_threshold;
    double probability;

    int ransac_max_iterations;
  } params_;

  // QoS Reliability Config
  std::string scan_reliability_;
  std::string line_cloud_pub_reliability_;
  std::string line_pose_pub_reliability_;
};

}  // namespace pcl_object_detection
