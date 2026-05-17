#include "pcl_object_detection/washing_machine_detection_component.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace pcl_object_detection {

WashingMachineDetectionComponent::WashingMachineDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("washing_machine_detection", options) {
  
  // ROI Parameters
  params_.x_min = this->declare_parameter<double>("roi.x_min", 0.3);
  params_.x_max = this->declare_parameter<double>("roi.x_max", 1.5);
  params_.y_min = this->declare_parameter<double>("roi.y_min", -0.7);
  params_.y_max = this->declare_parameter<double>("roi.y_max", 0.7);
  params_.z_min = this->declare_parameter<double>("roi.z_min", 0.1);
  params_.z_max = this->declare_parameter<double>("roi.z_max", 1.5);

  // Segmentation Parameters
  params_.plane_dist_threshold = this->declare_parameter<double>("plane.dist_threshold", 0.02);
  params_.circle_dist_threshold = this->declare_parameter<double>("circle.dist_threshold", 0.02);
  params_.circle_radius_min = this->declare_parameter<double>("circle.radius_min", 0.15);
  params_.circle_radius_max = this->declare_parameter<double>("circle.radius_max", 0.30);
  params_.smoothing_alpha = this->declare_parameter<double>("smoothing_alpha", 0.2);

  // TF Parameters
  params_.opening_axis_side = this->declare_parameter<std::string>("opening_axis_side", "left");
  params_.depth_shift = this->declare_parameter<double>("depth_shift", 0.0);
  params_.rotation_offset = this->declare_parameter<double>("rotation_offset", 0.0);
  params_.base_frame = this->declare_parameter<std::string>("base_frame", "base_footprint");

  cloud_raw_ = std::make_shared<PointCloud>();
  cloud_roi_ = std::make_shared<PointCloud>();
  cloud_plane_ = std::make_shared<PointCloud>();
  cloud_circle_ = std::make_shared<PointCloud>();
  debug_cloud_ = std::make_shared<PointCloud>();
}

WashingMachineDetectionComponent::CallbackReturn 
WashingMachineDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto qos = rclcpp::SensorDataQoS();
  std::string input_topic = this->declare_parameter<std::string>("input_topic", "cloud_filtered");
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    std::bind(&WashingMachineDetectionComponent::cloudCallback, this, std::placeholders::_1));

  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("washing_machine_debug_cloud", qos);

  RCLCPP_INFO(this->get_logger(), "Configured WashingMachineDetectionComponent");
  return CallbackReturn::SUCCESS;
}

WashingMachineDetectionComponent::CallbackReturn 
WashingMachineDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  pub_debug_cloud_->on_activate();
  RCLCPP_INFO(this->get_logger(), "Activated WashingMachineDetectionComponent");
  return CallbackReturn::SUCCESS;
}

WashingMachineDetectionComponent::CallbackReturn 
WashingMachineDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  pub_debug_cloud_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

WashingMachineDetectionComponent::CallbackReturn 
WashingMachineDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  sub_cloud_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

WashingMachineDetectionComponent::CallbackReturn 
WashingMachineDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

void WashingMachineDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (pub_debug_cloud_->get_subscription_count() == 0 && !tf_broadcaster_) return;

  // 1. Convert to PCL
  pcl::fromROSMsg(*msg, *cloud_raw_);
  if (cloud_raw_->empty()) return;

  // 2. ROI Filtering
  PointCloudUtility::applyPassThrough(cloud_raw_, cloud_roi_, "x", params_.x_min, params_.x_max);
  PointCloudUtility::applyPassThrough(cloud_roi_, cloud_roi_, "y", params_.y_min, params_.y_max);
  PointCloudUtility::applyPassThrough(cloud_roi_, cloud_roi_, "z", params_.z_min, params_.z_max);

  if (cloud_roi_->size() < 100) return;

  // 3. Plane Segmentation (Find the front face)
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.plane_dist_threshold);
  seg.setInputCloud(cloud_roi_);
  seg.segment(*inliers_plane, *coefficients_plane);

  if (inliers_plane->indices.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not estimate a planar model.");
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Plane found with %zu points. Coeffs: [%f, %f, %f, %f]", 
    inliers_plane->indices.size(), coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2], coefficients_plane->values[3]);

  // 4. Machine Isolation (Euclidean Clustering on the plane)
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi_);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);
  extract.filter(*cloud_plane_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.05); // 5cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);
  ec.setInputCloud(cloud_plane_);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No clusters found on plane.");
    return;
  }

  // Use the largest cluster (the washing machine face)
  pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices(cluster_indices[0]));
  extract.setInputCloud(cloud_plane_);
  extract.setIndices(largest_cluster);
  extract.filter(*cloud_plane_);

  // 5. Circle Detection
  // Project to plane to ensure perfection
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_plane_);
  proj.setModelCoefficients(coefficients_plane);
  proj.filter(*cloud_plane_);

  pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg_circle;
  seg_circle.setOptimizeCoefficients(true);
  seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
  seg_circle.setMethodType(pcl::SAC_RANSAC);
  seg_circle.setMaxIterations(2000);
  seg_circle.setDistanceThreshold(params_.circle_dist_threshold);
  seg_circle.setRadiusLimits(params_.circle_radius_min, params_.circle_radius_max);
  seg_circle.setInputCloud(cloud_plane_);
  seg_circle.segment(*inliers_circle, *coefficients_circle);

  if (inliers_circle->indices.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not find a circle on the machine face.");
    return;
  }

  // Circle params: [center_x, center_y, center_z, radius, normal_x, normal_y, normal_z]
  Eigen::Vector3d current_center(coefficients_circle->values[0], coefficients_circle->values[1], coefficients_circle->values[2]);
  double radius = coefficients_circle->values[3];
  
  // Plane Normal for orientation
  Eigen::Vector3d current_normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
  current_normal.normalize();
  if (current_normal.x() < 0) current_normal = -current_normal;

  // 6. Temporal Smoothing (EMA)
  if (is_first_detection_) {
    last_center_ = current_center;
    last_normal_ = current_normal;
    is_first_detection_ = false;
  } else {
    double alpha = params_.smoothing_alpha;
    last_center_ = alpha * current_center + (1.0 - alpha) * last_center_;
    last_normal_ = (alpha * current_normal + (1.0 - alpha) * last_normal_).normalized();
  }

  Eigen::Vector3d smoothed_center = last_center_;
  Eigen::Vector3d smoothed_normal = last_normal_;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Door Center (smoothed): [%f, %f, %f], Radius: %f",
    smoothed_center.x(), smoothed_center.y(), smoothed_center.z(), radius);

  // 7. TF Calculation: Placement ON THE RIM
  // 0-degree reference is "Top": Project World-Up onto the plane
  Eigen::Vector3d world_up(0, 0, 1);
  Eigen::Vector3d v_top = (world_up - (world_up.dot(smoothed_normal)) * smoothed_normal).normalized();
  Eigen::Vector3d v_right = smoothed_normal.cross(v_top).normalized();

  // Apply rotation offset (in degrees) to find the target rim position
  double offset_rad = params_.rotation_offset * M_PI / 180.0;
  Eigen::Vector3d v_target = std::cos(offset_rad) * v_top + std::sin(offset_rad) * v_right;

  // Final Rim Position P
  Eigen::Vector3d rim_pos = smoothed_center + radius * v_target;

  // Apply Depth Shift along normal
  rim_pos += params_.depth_shift * smoothed_normal;

  // Orientations at Rim Position:
  // X-axis: Points TO center (Radial Inward)
  Eigen::Vector3d x_axis = (smoothed_center - rim_pos).normalized();
  // Y-axis: Points OUT of door (Normal)
  Eigen::Vector3d y_axis = smoothed_normal;
  // Z-axis: Orthogonal (Tangential)
  Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

  // Construct Transform
  Eigen::Matrix3d m_rot;
  m_rot.col(0) = x_axis;
  m_rot.col(1) = y_axis;
  m_rot.col(2) = z_axis;
  Eigen::Quaterniond q(m_rot);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = msg->header.frame_id;
  t.child_frame_id = "washing_machine_door_rim";
  t.transform.translation.x = rim_pos.x();
  t.transform.translation.y = rim_pos.y();
  t.transform.translation.z = rim_pos.z();
  t.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(t);

  // Debug Cloud (the circle rim in 3D)
  if (pub_debug_cloud_->get_subscription_count() > 0) {
    pcl::ExtractIndices<PointT> extract_rim;
    extract_rim.setInputCloud(cloud_plane_);
    extract_rim.setIndices(inliers_circle);
    extract_rim.setNegative(false);
    extract_rim.filter(*cloud_circle_);
    
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*cloud_circle_, debug_msg);
    debug_msg.header = msg->header;
    pub_debug_cloud_->publish(debug_msg);
  }
}

} // namespace pcl_object_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::WashingMachineDetectionComponent)
