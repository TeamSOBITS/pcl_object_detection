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
#include <lifecycle_msgs/msg/state.hpp>

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
  params_.plane_vertical_eps_deg = this->declare_parameter<double>("plane.vertical_eps_deg", 20.0);
  params_.circle_dist_threshold = this->declare_parameter<double>("circle.dist_threshold", 0.02);
  params_.circle_radius_min = this->declare_parameter<double>("circle.radius_min", 0.15);
  params_.circle_radius_max = this->declare_parameter<double>("circle.radius_max", 0.30);
  params_.smoothing_alpha = this->declare_parameter<double>("smoothing_alpha", 0.2);

  // TF Parameters
  params_.opening_axis_side = this->declare_parameter<std::string>("opening_axis_side", "left");
  params_.depth_shift = this->declare_parameter<double>("depth_shift", 0.0);
  params_.rotation_offset = this->declare_parameter<double>("rotation_offset", 0.0);
  params_.base_frame = this->declare_parameter<std::string>("base_frame", "base_footprint");

  // Closed-drum front-face frame (door-closed approach aid).
  params_.closed_drum_enable = this->declare_parameter<bool>("closed_drum.enable", true);
  params_.closed_drum_frame = this->declare_parameter<std::string>("closed_drum.frame", "closed_drum");
  params_.closed_drum_min_align = this->declare_parameter<double>("closed_drum.min_align", 0.5);

  // QoS Parameters
  params_.cloud_reliability = this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  params_.debug_pub_reliability = this->declare_parameter<std::string>("debug_pub_reliability", "best_effort");

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

  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable"
      ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };

  std::string input_topic = this->declare_parameter<std::string>("input_topic", "filtered_cloud");

  // Explicit callback group so the executor reliably services this
  // subscription (created during a lifecycle transition in a shared container).
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, make_qos(params_.cloud_reliability, 10),
    std::bind(&WashingMachineDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "washing_machine_debug_cloud", make_qos(params_.debug_pub_reliability, 10));

  RCLCPP_INFO(this->get_logger(), "Cloud subscription reliability: %s", params_.cloud_reliability.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug publisher reliability: %s", params_.debug_pub_reliability.c_str());
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
  // Subscription is created in on_configure (so the container's executor collects
  // it), so gate processing on ACTIVE here to honour the lifecycle contract.
  if (this->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // Guard against a cloud callback that is in flight when on_cleanup() resets
  // these members (rapid activate→cleanup cycling): the ACTIVE state check above
  // is NOT atomic with this access — a callback can pass it and then race
  // on_cleanup nulling the publisher/broadcaster → null deref → container crash.
  // Bail out if either has already been torn down.
  if (!pub_debug_cloud_ || !tf_broadcaster_) return;
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
  // Constrain RANSAC to VERTICAL planes: the door face is vertical, so its
  // normal is HORIZONTAL (perpendicular to world up Z). An unconstrained
  // SACMODEL_PLANE grabs the largest flat region — on a real washer that's the
  // horizontal top/floor (normal ≈ ±Z), giving a meaningless "door normal" that
  // points up/down. SACMODEL_PERPENDICULAR_PLANE with axis = Z keeps only planes
  // whose normal is PERPENDICULAR to Z (i.e. vertical planes); eps is how far
  // (deg) the normal may tilt from horizontal.
  // SACMODEL_PERPENDICULAR_PLANE keeps planes whose normal is perpendicular to
  // the supplied axis (within eps). The door face is vertical → its normal is
  // perpendicular to world-up Z, so axis = Z selects vertical planes and rejects
  // the horizontal top/floor. eps = max tilt (deg) of the normal from horizontal.
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(params_.plane_vertical_eps_deg * M_PI / 180.0);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.plane_dist_threshold);
  seg.setInputCloud(cloud_roi_);
  seg.segment(*inliers_plane, *coefficients_plane);

  if (inliers_plane->indices.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Could not estimate a VERTICAL planar model (no door-like plane in ROI).");
    return;
  }
  // Defensive post-check: reject a fit whose normal is still too vertical (the
  // door plane normal must be near-horizontal). |nz| = |cos(angle from up)|.
  {
    const double nz = std::fabs(coefficients_plane->values[2]);
    const double max_nz = std::sin(params_.plane_vertical_eps_deg * M_PI / 180.0);
    if (nz > max_nz) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Rejected plane: normal too vertical (|nz|=%.3f > %.3f) — not the door face.",
        nz, max_nz);
      return;
    }
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

  // 4b. CLOSED-DRUM front-face frame. Broadcast HERE — from the front-plane fit,
  // BEFORE the porthole circle fit below — so it is published even when the door
  // is CLOSED (closed door → no visible porthole → the circle fit returns and we
  // never reach the rim broadcast). The front face centroid + plane normal give
  // ApproachWasher a true surface normal to square the base to the door, instead
  // of facing the SAM3 centroid along the robot→centroid ray.
  if (params_.closed_drum_enable) {
    broadcastClosedDrum(coefficients_plane, cloud_plane_, msg->header.frame_id,
                        this->get_clock()->now());
  }

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

void WashingMachineDetectionComponent::broadcastClosedDrum(
    const pcl::ModelCoefficients::ConstPtr & plane_coeffs,
    const PointCloud::ConstPtr & face_cloud,
    const std::string & cloud_frame,
    const rclcpp::Time & stamp) {
  if (face_cloud->empty()) return;

  // Front-face centroid (the "middle point of surface" anchor).
  Eigen::Vector4f c4;
  if (pcl::compute3DCentroid(*face_cloud, c4) == 0) return;
  Eigen::Vector3d center(c4.x(), c4.y(), c4.z());

  // Plane normal. Orient it INTO the machine: the cloud is in base_footprint
  // (robot at origin, +X forward) and the washer sits in front (center.x > 0),
  // so the inward normal points roughly +X. Flip so it points away from the
  // robot (from the face centroid towards +cloud-X away from origin).
  Eigen::Vector3d normal(plane_coeffs->values[0],
                         plane_coeffs->values[1],
                         plane_coeffs->values[2]);
  normal.normalize();
  // dir from robot origin to the face centroid (the "into machine" direction).
  Eigen::Vector3d to_face = center.normalized();
  if (normal.dot(to_face) < 0.0) normal = -normal;  // make normal point INTO machine

  // GATE: reject a side-wall plane. The FRONT face's inward normal must align
  // with the robot→face direction (front face faces the robot; a side wall's
  // normal is roughly perpendicular to that ray → low dot).
  const double align = normal.dot(to_face);
  if (align < params_.closed_drum_min_align) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "closed_drum: plane align %.2f < %.2f — likely a side wall, not the front face.",
      align, params_.closed_drum_min_align);
    return;
  }

  // EMA smoothing (separate state from the rim center/normal).
  if (is_first_drum_) {
    last_drum_center_ = center;
    last_drum_normal_ = normal;
    is_first_drum_ = false;
  } else {
    const double a = params_.smoothing_alpha;
    last_drum_center_ = a * center + (1.0 - a) * last_drum_center_;
    last_drum_normal_ = (a * normal + (1.0 - a) * last_drum_normal_).normalized();
  }

  // Frame: X = inward normal (heading the base should face), Z up-ish, Y = Z×X.
  Eigen::Vector3d x_axis = last_drum_normal_;
  Eigen::Vector3d world_up(0, 0, 1);
  Eigen::Vector3d z_axis = (world_up - world_up.dot(x_axis) * x_axis).normalized();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  Eigen::Matrix3d m_rot;
  m_rot.col(0) = x_axis;
  m_rot.col(1) = y_axis;
  m_rot.col(2) = z_axis;
  Eigen::Quaterniond q(m_rot);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = cloud_frame;
  t.child_frame_id = params_.closed_drum_frame;
  t.transform.translation.x = last_drum_center_.x();
  t.transform.translation.y = last_drum_center_.y();
  t.transform.translation.z = last_drum_center_.z();
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "closed_drum @ (%.3f, %.3f, %.3f) normal-into-machine align=%.2f",
    last_drum_center_.x(), last_drum_center_.y(), last_drum_center_.z(), align);
}

} // namespace pcl_object_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::WashingMachineDetectionComponent)
