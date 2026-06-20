#include "pcl_object_detection/laundry_detection_component.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/pca.h>
#include <lifecycle_msgs/msg/state.hpp>

namespace pcl_object_detection {

LaundryDetectionComponent::LaundryDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("laundry_detection", options) {
  
  // ROI Parameters
  params_.x_min = this->declare_parameter<double>("roi.x_min", 0.5);
  params_.x_max = this->declare_parameter<double>("roi.x_max", 2.5);
  params_.y_min = this->declare_parameter<double>("roi.y_min", -0.8);
  params_.y_max = this->declare_parameter<double>("roi.y_max", 0.8);
  params_.z_min = this->declare_parameter<double>("roi.z_min", 0.1);
  params_.z_max = this->declare_parameter<double>("roi.z_max", 1.5);

  // Segmentation Parameters
  params_.plane_dist_threshold = this->declare_parameter<double>("plane.dist_threshold", 0.03);
  params_.circle_dist_threshold = this->declare_parameter<double>("circle.dist_threshold", 0.02);
  params_.circle_radius_min = this->declare_parameter<double>("circle.radius_min", 0.18);
  params_.circle_radius_max = this->declare_parameter<double>("circle.radius_max", 0.26);

  // Drum Filtering
  params_.drum_depth = this->declare_parameter<double>("drum_depth", 0.50);
  params_.shell_threshold = this->declare_parameter<double>("shell_threshold", 0.04);
  
  // Clustering
  params_.cluster_tolerance = this->declare_parameter<double>("cluster_tolerance", 0.03);
  params_.cluster_min_size = this->declare_parameter<int>("cluster_min_size", 50);

  params_.base_frame = this->declare_parameter<std::string>("base_frame", "base_footprint");
  params_.input_topic = this->declare_parameter<std::string>("input_topic", "filtered_cloud");
  params_.min_machine_depth = this->declare_parameter<double>("min_machine_depth", 1.0);
  params_.drum_margin_front = this->declare_parameter<double>("drum_margin_front", 0.04);
  params_.drum_margin_back = this->declare_parameter<double>("drum_margin_back", 0.04);
  params_.min_laundry_depth = this->declare_parameter<double>("min_laundry_depth", 0.10);
  params_.smoothing_alpha = this->declare_parameter<double>("smoothing_alpha", 0.4);
  params_.voxel_size = this->declare_parameter<double>("voxel_size", 0.02);

  params_.cloud_reliability = this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  params_.drum_pub_reliability = this->declare_parameter<std::string>("drum_pub_reliability", "best_effort");
  params_.laundry_pub_reliability = this->declare_parameter<std::string>("laundry_pub_reliability", "best_effort");

  cloud_raw_ = std::make_shared<PointCloud>();
  cloud_roi_ = std::make_shared<PointCloud>();
  cloud_drum_ = std::make_shared<PointCloud>();
  cloud_laundry_ = std::make_shared<PointCloud>();
}

LaundryDetectionComponent::CallbackReturn
LaundryDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->get_parameter("input_topic", params_.input_topic);
  this->get_parameter("base_frame", params_.base_frame);
  this->get_parameter("roi.x_min", params_.x_min);
  this->get_parameter("roi.x_max", params_.x_max);
  this->get_parameter("roi.y_min", params_.y_min);
  this->get_parameter("roi.y_max", params_.y_max);
  this->get_parameter("roi.z_min", params_.z_min);
  this->get_parameter("roi.z_max", params_.z_max);
  this->get_parameter("plane.dist_threshold", params_.plane_dist_threshold);
  this->get_parameter("circle.dist_threshold", params_.circle_dist_threshold);
  this->get_parameter("circle.radius_min", params_.circle_radius_min);
  this->get_parameter("circle.radius_max", params_.circle_radius_max);
  this->get_parameter("drum_depth", params_.drum_depth);
  this->get_parameter("shell_threshold", params_.shell_threshold);
  this->get_parameter("cluster_tolerance", params_.cluster_tolerance);
  this->get_parameter("cluster_min_size", params_.cluster_min_size);
  this->get_parameter("min_machine_depth", params_.min_machine_depth);
  this->get_parameter("drum_margin_front", params_.drum_margin_front);
  this->get_parameter("drum_margin_back", params_.drum_margin_back);
  this->get_parameter("min_laundry_depth", params_.min_laundry_depth);
  this->get_parameter("smoothing_alpha", params_.smoothing_alpha);
  this->get_parameter("voxel_size", params_.voxel_size);

  // Allow hot-reloading of tunable params without lifecycle restart.
  // topic/frame are excluded as they require re-creating the subscription.
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult {
      for (const auto & p : params) {
        const auto & n = p.get_name();
        if      (n == "plane.dist_threshold")  params_.plane_dist_threshold  = p.as_double();
        else if (n == "circle.dist_threshold") params_.circle_dist_threshold = p.as_double();
        else if (n == "circle.radius_min")     params_.circle_radius_min     = p.as_double();
        else if (n == "circle.radius_max")     params_.circle_radius_max     = p.as_double();
        else if (n == "drum_depth")            params_.drum_depth            = p.as_double();
        else if (n == "shell_threshold")       params_.shell_threshold       = p.as_double();
        else if (n == "cluster_tolerance")     params_.cluster_tolerance     = p.as_double();
        else if (n == "cluster_min_size")      params_.cluster_min_size      = static_cast<int>(p.as_int());
        else if (n == "min_machine_depth")     params_.min_machine_depth     = p.as_double();
        else if (n == "drum_margin_front")     params_.drum_margin_front     = p.as_double();
        else if (n == "drum_margin_back")      params_.drum_margin_back      = p.as_double();
        else if (n == "min_laundry_depth")     params_.min_laundry_depth     = p.as_double();
        else if (n == "smoothing_alpha")       params_.smoothing_alpha       = p.as_double();
        else if (n == "voxel_size")            params_.voxel_size            = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable"
      ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };

  last_process_time_ = this->get_clock()->now();
  // Explicit reentrant callback group so the cloud subscription is reliably
  // serviced by the component container's multithreaded executor (see header).
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create the cloud subscription HERE in on_configure, NOT in on_activate.
  // Adding a subscription to a node that the component container's executor is
  // ALREADY spinning does not rebuild the executor's wait set, so the callback
  // never fires. Creating it during configure (before the container starts
  // servicing this node) gets it collected into the wait set. We gate actual
  // processing on the lifecycle state instead (cloudCallback returns early
  // unless active), so no clouds are processed while inactive.
  {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, make_qos(params_.cloud_reliability, 10),
      std::bind(&LaundryDetectionComponent::cloudCallback, this, std::placeholders::_1),
      sub_opts);
  }

  pub_drum_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("drum_debug_cloud", make_qos(params_.drum_pub_reliability, 10));
  pub_laundry_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laundry_debug_cloud", make_qos(params_.laundry_pub_reliability, 10));

  RCLCPP_INFO(this->get_logger(), "QoS reliability — cloud: %s, drum_pub: %s, laundry_pub: %s",
    params_.cloud_reliability.c_str(), params_.drum_pub_reliability.c_str(), params_.laundry_pub_reliability.c_str());
  RCLCPP_INFO(this->get_logger(), "Configured LaundryDetectionComponent");
  return CallbackReturn::SUCCESS;
}

LaundryDetectionComponent::CallbackReturn
LaundryDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  pub_drum_cloud_->on_activate();
  pub_laundry_cloud_->on_activate();
  last_process_time_ = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Activated LaundryDetectionComponent");
  return CallbackReturn::SUCCESS;
}

LaundryDetectionComponent::CallbackReturn
LaundryDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  pub_drum_cloud_->on_deactivate();
  pub_laundry_cloud_->on_deactivate();
  smoothed_centroid_initialized_ = false;
  smoothed_entrance_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

LaundryDetectionComponent::CallbackReturn 
LaundryDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  sub_cloud_.reset();
  pub_drum_cloud_.reset();
  pub_laundry_cloud_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

LaundryDetectionComponent::CallbackReturn 
LaundryDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

void LaundryDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Subscription is created in on_configure (so the container's executor collects
  // it), so gate processing on ACTIVE here to honour the lifecycle contract.
  if (this->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  auto start_time = this->get_clock()->now();

  // Throttle to ~5 Hz.
  const int64_t period_ns = static_cast<int64_t>(1e9 / 5.0);
  // If the throttle baseline and now have different clock sources (SYSTEM vs ROS,
  // e.g. before use_sim_time syncs), subtracting them throws — back-date the
  // baseline so this frame processes instead of crashing or being dropped.
  if (last_process_time_.get_clock_type() != start_time.get_clock_type()) {
    last_process_time_ = start_time - rclcpp::Duration::from_nanoseconds(period_ns + 1);
  }
  if ((start_time - last_process_time_).nanoseconds() < period_ns) {
    return;
  }
  last_process_time_ = start_time;

  // Helper: re-publish the last known good TF so the frame doesn't expire in the
  // TF buffer during frames where the pipeline fails partway through.
  auto republish_last_known = [&]() {
    if (!smoothed_centroid_initialized_) return;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = "laundry_item";
    t.transform.translation.x = smoothed_centroid_.x();
    t.transform.translation.y = smoothed_centroid_.y();
    t.transform.translation.z = smoothed_centroid_.z();
    t.transform.rotation = last_known_rotation_;
    tf_broadcaster_->sendTransform(t);
  };

  // 1.5 Convert to PCL into a local cloud to avoid data races between callbacks.
  PointCloud::Ptr cloud_roi(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud_roi);
  if (cloud_roi->empty()) { republish_last_known(); return; }
  if (cloud_roi->size() < 10) { republish_last_known(); return; }

  // 3. Find Machine Front Plane (Skipping closer planes like an open door)
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  PointCloud::Ptr cloud_remaining(new PointCloud(*cloud_roi));

  bool plane_found = false;
  for (int i = 0; i < 3; ++i) { // Try up to 3 major planes
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200); // Cap iterations to bound processing time
    seg.setDistanceThreshold(params_.plane_dist_threshold);
    seg.setInputCloud(cloud_remaining);
    seg.segment(*inliers_plane, *coefficients_plane);

    if (inliers_plane->indices.empty()) break;

    // Calculate centroid depth
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_remaining, *inliers_plane, centroid);

    // Calculate plane normal direction
    Eigen::Vector3d plane_normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
    plane_normal.normalize();
    double alignment = std::abs(plane_normal.dot(Eigen::Vector3d(1, 0, 0))); // Alignment with robot X axis

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Plane %d: depth X=%.3f, alignment=%.3f%s", i, centroid[0], alignment,
      (centroid[0] >= params_.min_machine_depth && alignment > 0.8) ? " -> ACCEPTED" : " -> rejected");

    // We want a plane that is at the correct depth AND facing the robot
    if (centroid[0] >= params_.min_machine_depth && alignment > 0.8) {
      plane_found = true;
      break;
    }

    // Otherwise, this plane is too close or misaligned (e.g. open door, floor). Remove and retry.
    pcl::ExtractIndices<PointT> extract_door;
    extract_door.setInputCloud(cloud_remaining);
    extract_door.setIndices(inliers_plane);
    extract_door.setNegative(true);
    PointCloud::Ptr tmp(new PointCloud);
    extract_door.filter(*tmp);
    cloud_remaining = tmp;
  }

  if (!plane_found) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not find a plane at the required depth.");
    republish_last_known();
    return;
  }

  // 4. Find Opening Circle
  // Extract the accepted plane's inlier points from cloud_remaining (the cloud
  // that was active when segment() ran — inlier indices index into it, not cloud_roi_).
  PointCloud::Ptr cloud_plane(new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_remaining);
  extract.setIndices(inliers_plane);
  extract.filter(*cloud_plane);

  pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg_circle;
  seg_circle.setOptimizeCoefficients(true);
  seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
  seg_circle.setMethodType(pcl::SAC_RANSAC);
  seg_circle.setMaxIterations(1000);
  seg_circle.setDistanceThreshold(params_.circle_dist_threshold);
  seg_circle.setRadiusLimits(params_.circle_radius_min, params_.circle_radius_max);
  seg_circle.setInputCloud(cloud_plane);
  seg_circle.segment(*inliers_circle, *coefficients_circle);
  
  if (inliers_circle->indices.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Circle fitting failed.");
    republish_last_known();
    return;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    "Circle found with radius: %f", coefficients_circle->values[3]);

  // Circle params: [center_x, center_y, center_z, radius, normal_x, normal_y, normal_z]
  Eigen::Vector3d center(coefficients_circle->values[0], coefficients_circle->values[1], coefficients_circle->values[2]);
  double radius = coefficients_circle->values[3];
  Eigen::Vector3d normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
  normal.normalize();
  
  // Ensure drum_dir points away from the robot (into the machine, along +X usually)
  Eigen::Vector3d drum_dir = normal;
  if (drum_dir.x() < 0) drum_dir = -drum_dir;

  // Note: the front face normal would be -drum_dir (pointing towards robot)

  // Shared drum-frame orientation (X into drum, Y horizontal along opening, Z up).
  // Reused for both the drum_entrance and laundry_item frames.
  Eigen::Vector3d x_axis = drum_dir.normalized();
  Eigen::Vector3d y_axis = Eigen::Vector3d(0, 0, 1).cross(x_axis).normalized();
  Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
  Eigen::Matrix3d drum_rot;
  drum_rot.col(0) = x_axis;
  drum_rot.col(1) = y_axis;
  drum_rot.col(2) = z_axis;
  geometry_msgs::msg::Quaternion drum_quat = tf2::toMsg(Eigen::Quaterniond(drum_rot));

  // Entrance center: use the RANSAC circle center (`center`) — it estimates the
  // true geometric center of the opening from the rim arc, so it is NOT biased
  // toward whichever arc the (downward-pitched) camera happens to see. The raw
  // estimate is jittery frame-to-frame, so EMA-smooth it to damp bad frames.
  // (The rim-point centroid was tried but sits biased upward, since the camera
  // sees the upper rim better than the self-occluded lower rim.)
  Eigen::Vector3d entrance = center;
  if (!smoothed_entrance_initialized_) {
    smoothed_entrance_ = entrance;
    smoothed_entrance_initialized_ = true;
  } else {
    smoothed_entrance_ = params_.smoothing_alpha * entrance +
                         (1.0 - params_.smoothing_alpha) * smoothed_entrance_;
  }

  // Broadcast the drum entrance every frame a circle is found, independent of
  // whether laundry is detected inside — lets the FSM aim at the opening even
  // when the drum is empty.
  // NOTE: this is an APPROACH aid, not a metrically exact opening pose. RANSAC
  // tends to fit a slightly oversized circle (porthole blends into the flat
  // face), so the centre sits ~0.15 m above the true drum centre. Use it for
  // approach direction; rely on laundry_item for the actual grasp target.
  {
    geometry_msgs::msg::TransformStamped te;
    te.header.stamp = this->get_clock()->now();
    te.header.frame_id = msg->header.frame_id;
    te.child_frame_id = "drum_entrance";
    te.transform.translation.x = smoothed_entrance_.x();
    te.transform.translation.y = smoothed_entrance_.y();
    te.transform.translation.z = smoothed_entrance_.z();
    te.transform.rotation = drum_quat;
    tf_broadcaster_->sendTransform(te);
  }

  // 5. Cylindrical Volume Extraction
  cloud_drum_->clear();
  for (const auto& point : cloud_roi->points) {
    Eigen::Vector3d p(point.x, point.y, point.z);
    Eigen::Vector3d rel_p = p - center;
    
    double dist_into_drum = rel_p.dot(drum_dir);
    Eigen::Vector3d p_proj = rel_p - dist_into_drum * drum_dir;
    double dist_from_axis = p_proj.norm();

    // Check if inside drum but not touching walls/back
    // Using drum_margin_front to skip the machine surface and drum_margin_back to skip the rear wall
    if (dist_into_drum > params_.drum_margin_front && dist_into_drum < params_.drum_depth - params_.drum_margin_back &&
        dist_from_axis < radius - params_.shell_threshold) {
      cloud_drum_->push_back(point);
    }
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    "Extracted %zu points into the drum volume.", cloud_drum_->size());

  if (cloud_drum_->empty()) { republish_last_known(); return; }

  // 6. Cluster Laundry
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(params_.cluster_tolerance);
  ec.setMinClusterSize(params_.cluster_min_size);
  ec.setMaxClusterSize(25000);
  ec.setInputCloud(cloud_drum_);
  ec.extract(cluster_indices);

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    "Clustering found %zu clusters within the drum.", cluster_indices.size());

  if (cluster_indices.empty()) { republish_last_known(); return; }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    "Largest cluster size: %zu", cluster_indices[0].indices.size());

  // Use the largest cluster as our laundry pile
  pcl::PointIndices::Ptr laundry_inliers(new pcl::PointIndices(cluster_indices[0]));
  extract.setInputCloud(cloud_drum_);
  extract.setIndices(laundry_inliers);
  extract.filter(*cloud_laundry_);

  // 7. Calculate Centroid for TF
  Eigen::Vector4f laundry_centroid;
  pcl::compute3DCentroid(*cloud_laundry_, laundry_centroid);

  // Reject if centroid is too close to the drum opening (likely rim/counter noise)
  Eigen::Vector3d centroid_3d(laundry_centroid[0], laundry_centroid[1], laundry_centroid[2]);
  double centroid_depth = (centroid_3d - center).dot(drum_dir);
  if (centroid_depth < params_.min_laundry_depth) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Laundry centroid depth %.3f m is shallower than min_laundry_depth %.3f m — likely rim noise, skipping.",
      centroid_depth, params_.min_laundry_depth);
    republish_last_known();
    return;
  }

  // Exponential moving average to smooth out frame-to-frame centroid jitter.
  // alpha=1.0 → no smoothing (raw centroid); alpha→0.0 → very heavy smoothing.
  if (!smoothed_centroid_initialized_) {
    smoothed_centroid_ = centroid_3d;
    smoothed_centroid_initialized_ = true;
  } else {
    smoothed_centroid_ = params_.smoothing_alpha * centroid_3d +
                         (1.0 - params_.smoothing_alpha) * smoothed_centroid_;
  }

  // 8. Build and publish TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = msg->header.frame_id;
  t.child_frame_id = "laundry_item";
  t.transform.translation.x = smoothed_centroid_.x();
  t.transform.translation.y = smoothed_centroid_.y();
  t.transform.translation.z = smoothed_centroid_.z();

  // Reuse the drum-frame orientation (stable, from the fitted plane normal)
  // rather than PCA on the messy laundry blob, which is noisy and ambiguous.
  last_known_rotation_ = drum_quat;
  t.transform.rotation = last_known_rotation_;

  tf_broadcaster_->sendTransform(t);

  auto end_time = this->get_clock()->now();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "Laundry detection took %.3f s. Found laundry at X: %.3f (depth in drum: %.3f m)",
    (end_time - start_time).seconds(), t.transform.translation.x, centroid_depth);

  // Debug Clouds
  if (pub_drum_cloud_->is_activated()) {
    sensor_msgs::msg::PointCloud2 drum_msg;
    pcl::toROSMsg(*cloud_drum_, drum_msg);
    drum_msg.header = msg->header;
    pub_drum_cloud_->publish(drum_msg);
  }
  if (pub_laundry_cloud_->is_activated()) {
    sensor_msgs::msg::PointCloud2 laundry_msg;
    pcl::toROSMsg(*cloud_laundry_, laundry_msg);
    laundry_msg.header = msg->header;
    pub_laundry_cloud_->publish(laundry_msg);
  }
}

} // namespace pcl_object_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::LaundryDetectionComponent)
