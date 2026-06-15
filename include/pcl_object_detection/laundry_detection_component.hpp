#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/quaternion.hpp>

#include "pcl_object_detection/point_cloud_utility.hpp"

namespace pcl_object_detection {

class LaundryDetectionComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LaundryDetectionComponent(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  struct Parameters {
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    double plane_dist_threshold;
    double circle_dist_threshold;
    double circle_radius_min;
    double circle_radius_max;
    double drum_depth;
    double shell_threshold;
    double cluster_tolerance;
    int cluster_min_size;
    double min_machine_depth;
    double drum_margin_front;
    double drum_margin_back;
    double min_laundry_depth;
    double smoothing_alpha;
    double voxel_size;
    
    std::string input_topic;
    std::string base_frame;
    std::string cloud_reliability;
    std::string drum_pub_reliability;
    std::string laundry_pub_reliability;
  } params_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_drum_cloud_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laundry_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Time last_process_time_;

  // EMA smoothing state and last-known TF for keep-alive republishing
  Eigen::Vector3d smoothed_centroid_{0, 0, 0};
  bool smoothed_centroid_initialized_{false};
  geometry_msgs::msg::Quaternion last_known_rotation_{};

  // PCL
  PointCloud::Ptr cloud_raw_;
  PointCloud::Ptr cloud_roi_;
  PointCloud::Ptr cloud_drum_;
  PointCloud::Ptr cloud_laundry_;
};

} // namespace pcl_object_detection
