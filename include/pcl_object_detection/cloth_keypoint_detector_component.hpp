#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>

#include "pcl_object_detection/point_cloud_utility.hpp"

namespace pcl_object_detection {

/**
 * Listens to the clustered cloth point cloud from mask_to_3d (already in
 * base_footprint) and computes three grasp-point TF frames for the Japanese
 * Fast Fold:
 *
 *   cloth_fold_p1  – near-left  OBB corner  (right arm first grasp)
 *   cloth_fold_p2  – near-right OBB corner  (left  arm first grasp)
 *   cloth_fold_p3  – far-left   OBB corner  (right arm crosses to)
 *
 * "Near" = smaller X in base_footprint (closer to robot).
 * All three points are lifted by z_grasp_offset above the cloud's max Z so
 * the grippers contact cloth, not table surface.
 */
class ClothKeypointDetectorComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ClothKeypointDetectorComponent(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void broadcastPoint(const std::string & frame_id,
                      double x, double y, double z,
                      const rclcpp::Time & stamp);

  struct Parameters {
    std::string cloud_topic;
    std::string base_frame;
    double z_grasp_offset;
    int    min_points;
    double publish_hz;
    // "right" or "left" — which half of the shirt to place the fold column on.
    std::string fold_side;
    // Fraction of shirt width to inset from each edge before computing the fold column.
    // Removes sleeve area from the minor-axis extent used for column placement.
    double sleeve_inset_fraction;
    // Percentile (0–50) used for PB/PC X position within the column band.
    // e.g. 2 → PB at 98th percentile, PC at 2nd percentile of column-local X values.
    double edge_percentile;
    // Fraction of shirt width used as the ±band around col_y when scanning for PB/PC X extent.
    // Scales with shirt size; 0.25 = ±25% of shirt width, min clamped to 3 cm.
    double col_band_fraction;
    // QoS reliability for the cloud subscriber: "reliable" or "best_effort".
    std::string cloud_reliability;
    // QoS reliability for the debug cloud publisher: "reliable" or "best_effort".
    std::string debug_pub_reliability;
  } params_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_cloud_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Time last_process_time_;
};

}  // namespace pcl_object_detection
