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
#include <deque>
#include <vector>
#include <algorithm>

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
    int circle_min_inliers;  // min plane points before attempting CIRCLE3D fit
                             // (guards PCL collinear-sample stderr flood)
    double drum_depth;
    double shell_threshold;
    double cluster_tolerance;
    int cluster_min_size;
    double min_machine_depth;
    double drum_margin_front;
    double drum_margin_back;
    double min_laundry_depth;
    double smoothing_alpha;          // EMA weight for the laundry_item centroid
    double entrance_smoothing_alpha; // EMA weight for drum_entrance (heavier: the
                                     // RANSAC circle centre jitters in y, and the
                                     // opening is static during approach)
    int entrance_median_window;      // sliding-window median size applied to the
                                     // raw circle centre before the EMA, to reject
                                     // the broadband ±0.13 m lateral RANSAC jitter
    double voxel_size;

    // Cavity-validation gates: reject phantom laundry_item on a CLOSED door.
    // Closed door has no porthole → RANSAC fits side wall, CIRCLE3D over-fits a
    // coplanar circle off-axis (measured centre y=+0.535), wall points cluster
    // into a fake pile. These gates reject that geometry; a real open drum
    // passes.
    double cavity_expected_machine_y;    // expected lateral porthole centre
    double cavity_machine_center_tol_y;  // |center.y - expected| reject band
    double cavity_center_x_min;          // accept circle centre X in [min,max]
    double cavity_center_x_max;
    double cavity_center_z_min;          // accept circle centre Z in [min,max]
    double cavity_center_z_max;
    double cavity_min_axial_spread;      // min depth-into-drum extent of cluster
    int    cavity_min_drum_volume_points;// min points carved into drum volume
    double cavity_min_cluster_fraction;  // largest cluster / drum volume points

    // drum_entrance lateral/vertical correction: RANSAC fits only a partial
    // porthole arc, biasing the circle centre toward the visible arc (measured
    // ~0.046 m right, ~0.03 m high vs the symmetric drum-cloud centre). Apply a
    // fixed offset in base_footprint so drum_entrance lands on the true centre.
    double entrance_y_offset;  // (m) +Y = robot left
    double entrance_z_offset;  // (m) +Z = up

    std::string input_topic;
    std::string base_frame;
    std::string cloud_reliability;
    std::string drum_pub_reliability;
    std::string laundry_pub_reliability;
  } params_;

  // ROS
  // Dedicated callback group for the cloud subscription. Without an explicit
  // group, a plain subscription created in on_configure() on a LifecycleNode
  // running inside a component_container_mt can fail to be added to the
  // executor's wait set, so its callback never fires even though messages and
  // QoS are fine. Owning the group guarantees the executor services it.
  rclcpp::CallbackGroup::SharedPtr cb_group_;
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
  Eigen::Vector3d smoothed_entrance_{0, 0, 0};
  bool smoothed_entrance_initialized_{false};
  std::deque<Eigen::Vector3d> entrance_history_;  // raw circle centres for median
  geometry_msgs::msg::Quaternion last_known_rotation_{};

  // PCL
  PointCloud::Ptr cloud_raw_;
  PointCloud::Ptr cloud_roi_;
  PointCloud::Ptr cloud_drum_;
  PointCloud::Ptr cloud_laundry_;
};

} // namespace pcl_object_detection
