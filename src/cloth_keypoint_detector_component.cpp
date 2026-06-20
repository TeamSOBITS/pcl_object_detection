#include "pcl_object_detection/cloth_keypoint_detector_component.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <algorithm>

namespace pcl_object_detection {

ClothKeypointDetectorComponent::ClothKeypointDetectorComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("cloth_keypoint_detector", options)
{
  params_.cloud_topic           = this->declare_parameter<std::string>("cloud_topic", "/mask_to_3d/object_3d_cloud");
  params_.base_frame            = this->declare_parameter<std::string>("base_frame",  "base_footprint");
  params_.z_grasp_offset        = this->declare_parameter<double>("z_grasp_offset", 0.015);
  params_.min_points            = this->declare_parameter<int>("min_points", 200);
  params_.publish_hz            = this->declare_parameter<double>("publish_hz", 5.0);
  params_.fold_side             = this->declare_parameter<std::string>("fold_side", "right");
  params_.sleeve_inset_fraction = this->declare_parameter<double>("sleeve_inset_fraction", 0.20);
  params_.edge_percentile       = this->declare_parameter<double>("edge_percentile", 5.0);
  params_.col_band_fraction     = this->declare_parameter<double>("col_band_fraction", 0.25);
  params_.cloud_reliability     = this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  params_.debug_pub_reliability = this->declare_parameter<std::string>("debug_pub_reliability", "best_effort");
}

ClothKeypointDetectorComponent::CallbackReturn
ClothKeypointDetectorComponent::on_configure(const rclcpp_lifecycle::State &)
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      const bool is_active =
        (this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      for (const auto & p : params) {
        const auto & n = p.get_name();
        // Live-tunable params (take effect immediately)
        if      (n == "z_grasp_offset")     params_.z_grasp_offset     = p.as_double();
        else if (n == "min_points")         params_.min_points         = static_cast<int>(p.as_int());
        else if (n == "edge_percentile")    params_.edge_percentile    = p.as_double();
        else if (n == "col_band_fraction")  params_.col_band_fraction  = p.as_double();
        // Params that require deactivate → configure → activate to take effect
        else if (n == "cloud_topic" || n == "cloud_reliability" ||
                 n == "debug_pub_reliability") {
          if (is_active) {
            r.successful = false;
            r.reason = n + " cannot be changed while active — deactivate, cleanup, then configure";
            return r;
          }
        }
      }
      return r;
    });

  // Re-read connection params — they may have been changed while inactive via ros2 param set.
  params_.cloud_topic          = this->get_parameter("cloud_topic").as_string();
  params_.cloud_reliability     = this->get_parameter("cloud_reliability").as_string();
  params_.debug_pub_reliability = this->get_parameter("debug_pub_reliability").as_string();

  auto pub_reliability = (params_.debug_pub_reliability == "reliable")
    ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
    : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(pub_reliability);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "cloth_keypoint_detector/debug_cloud", qos);
  RCLCPP_INFO(this->get_logger(), "ClothKeypointDetectorComponent configured:");
  RCLCPP_INFO(this->get_logger(), "  cloud_topic:           %s", params_.cloud_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_frame:            %s", params_.base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  z_grasp_offset:        %.4f", params_.z_grasp_offset);
  RCLCPP_INFO(this->get_logger(), "  min_points:            %d", params_.min_points);
  RCLCPP_INFO(this->get_logger(), "  publish_hz:            %.2f", params_.publish_hz);
  RCLCPP_INFO(this->get_logger(), "  fold_side:             %s", params_.fold_side.c_str());
  RCLCPP_INFO(this->get_logger(), "  sleeve_inset_fraction: %.3f", params_.sleeve_inset_fraction);
  RCLCPP_INFO(this->get_logger(), "  edge_percentile:       %.2f", params_.edge_percentile);
  RCLCPP_INFO(this->get_logger(), "  col_band_fraction:     %.2f", params_.col_band_fraction);
  RCLCPP_INFO(this->get_logger(), "  cloud_reliability:     %s", params_.cloud_reliability.c_str());
  RCLCPP_INFO(this->get_logger(), "  debug_pub_reliability: %s", params_.debug_pub_reliability.c_str());
  return CallbackReturn::SUCCESS;
}

ClothKeypointDetectorComponent::CallbackReturn
ClothKeypointDetectorComponent::on_activate(const rclcpp_lifecycle::State &)
{
  pub_debug_cloud_->on_activate();

  auto reliability = (params_.cloud_reliability == "reliable")
    ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
    : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(reliability);

  // Explicit callback group so the executor reliably services this
  // subscription (created during a lifecycle transition in a shared container).
  if (!cb_group_) {
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.cloud_topic, qos,
    std::bind(&ClothKeypointDetectorComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  last_process_time_ = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Activated ClothKeypointDetectorComponent");
  return CallbackReturn::SUCCESS;
}

ClothKeypointDetectorComponent::CallbackReturn
ClothKeypointDetectorComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  sub_cloud_.reset();  // stop processing immediately
  pub_debug_cloud_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

ClothKeypointDetectorComponent::CallbackReturn
ClothKeypointDetectorComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

ClothKeypointDetectorComponent::CallbackReturn
ClothKeypointDetectorComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void ClothKeypointDetectorComponent::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Throttle to publish_hz.
  auto now = this->get_clock()->now();
  const int64_t period_ns = static_cast<int64_t>(1e9 / params_.publish_hz);
  // If the throttle baseline and now have different clock sources (SYSTEM vs ROS,
  // e.g. before use_sim_time syncs), subtracting them throws — back-date the
  // baseline so this frame processes instead of crashing or being dropped.
  if (last_process_time_.get_clock_type() != now.get_clock_type()) {
    last_process_time_ = now - rclcpp::Duration::from_nanoseconds(period_ns + 1);
  }
  if ((now - last_process_time_).nanoseconds() < period_ns) {
    return;
  }
  last_process_time_ = now;

  // Convert
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);

  if (static_cast<int>(cloud->size()) < params_.min_points) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cloth cloud too small (%zu pts, need %d) — skipping keypoint update.",
      cloud->size(), params_.min_points);
    return;
  }

  // Build a fake PointIndices spanning the whole cloud for computePCAAlignedBox
  pcl::PointIndices all_indices;
  all_indices.indices.resize(cloud->size());
  std::iota(all_indices.indices.begin(), all_indices.indices.end(), 0);

  // Compute OBB via existing utility (PCA in XY, Z from min/max)
  vision_msgs::msg::BoundingBox3D box =
    PointCloudUtility::computePCAAlignedBox(cloud, all_indices);

  // Extract OBB corner coordinates from box.
  // box.center is the centroid; box.size.x = major (length), box.size.y = minor (width).
  // The OBB yaw is encoded in box.center.orientation as a quaternion (rotation around Z).
  // Extract yaw angle from quaternion: yaw = 2 * atan2(qz, qw)
  const double qz  = box.center.orientation.z;
  const double qw  = box.center.orientation.w;
  const double yaw = 2.0 * std::atan2(qz, qw);

  const double cx = box.center.position.x;
  const double cy = box.center.position.y;

  // Unit vectors of the OBB axes in base_footprint XY
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const Eigen::Vector2d major_ax( cos_y,  sin_y);
  const Eigen::Vector2d minor_ax(-sin_y,  cos_y);

  // Project every point onto OBB axes and collect per-slice minor extents.
  // We divide the shirt along the major axis into three bands (near / mid / far)
  // and track the actual minor-axis range within each band.  This prevents
  // keypoints from falling outside the cloud at the collar (which is narrower
  // than the shirt body so the global pmin_max overshoots at pmaj_max).
  double pmaj_min =  std::numeric_limits<double>::max();
  double pmaj_max = -std::numeric_limits<double>::max();
  double pmin_min =  std::numeric_limits<double>::max();
  double pmin_max = -std::numeric_limits<double>::max();
  for (const auto & pt : cloud->points) {
    Eigen::Vector2d v(pt.x - cx, pt.y - cy);
    double dmaj = v.dot(major_ax);
    double dmin = v.dot(minor_ax);
    if (dmaj < pmaj_min) pmaj_min = dmaj;
    if (dmaj > pmaj_max) pmaj_max = dmaj;
    if (dmin < pmin_min) pmin_min = dmin;
    if (dmin > pmin_max) pmin_max = dmin;
  }

  const double pmaj_mid = (pmaj_min + pmaj_max) * 0.5;
  // Band half-width for column sampling: ⅙ of shirt length centred on mid-body.
  const double band_hw  = (pmaj_max - pmaj_min) / 6.0;

  // Global major-axis percentile (fallback for PB/PC; column-local pass is below).
  // edge_percentile excludes outlier fringe points while keeping the shirt length.
  std::vector<double> dmaj_vals;
  dmaj_vals.reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    Eigen::Vector2d v(pt.x - cx, pt.y - cy);
    dmaj_vals.push_back(v.dot(major_ax));
  }
  std::sort(dmaj_vals.begin(), dmaj_vals.end());
  const double pct_lo = params_.edge_percentile / 100.0;
  const double pct_hi = 1.0 - pct_lo;
  const double pmaj_near_dense = dmaj_vals[static_cast<size_t>(pct_lo * (dmaj_vals.size() - 1))];
  const double pmaj_far_dense  = dmaj_vals[static_cast<size_t>(pct_hi * (dmaj_vals.size() - 1))];

  // Compute shared column in WORLD Y from the mid-body band (widest, most reliable).
  // Using world Y directly guarantees all three TFs share the exact same Y value —
  // a constant OBB minor projection maps to slightly varying world Y when yaw != 0.
  double world_y_max_mid = -std::numeric_limits<double>::max();
  double world_y_min_mid =  std::numeric_limits<double>::max();
  for (const auto & pt : cloud->points) {
    Eigen::Vector2d v(pt.x - cx, pt.y - cy);
    double dmaj = v.dot(major_ax);
    if (std::abs(dmaj - pmaj_mid) <= band_hw) {
      if (pt.y > world_y_max_mid) world_y_max_mid = pt.y;
      if (pt.y < world_y_min_mid) world_y_min_mid = pt.y;
    }
  }
  if (world_y_max_mid < world_y_min_mid) {
    world_y_max_mid = cy + pmin_max;
    world_y_min_mid = cy + pmin_min;
  }

  // Inset from both edges by sleeve_inset_fraction to exclude sleeve area.
  // In base_footprint: +Y = left, -Y = right.
  const double shirt_width = world_y_max_mid - world_y_min_mid;
  const double inset = shirt_width * params_.sleeve_inset_fraction;
  const double body_y_left  = world_y_max_mid - inset;   // inset from left  (sleeve removed)
  const double body_y_right = world_y_min_mid + inset;   // inset from right (sleeve removed)
  const double body_centre_y = (body_y_left + body_y_right) * 0.5;

  // Quarter column on the chosen side: midpoint of (body-centre, chosen-body-edge).
  const double col_y = (params_.fold_side == "left")
    ? (body_centre_y + body_y_left)  * 0.5
    : (body_centre_y + body_y_right) * 0.5;

  // Minor-axis offset placing the column at world Y = col_y:
  //   cy + dmin_col * minor_ax.y() = col_y  →  dmin_col = (col_y - cy) / minor_ax.y()
  // If minor_ax.y() ≈ 0 the major axis is mostly Y (fold column runs along X),
  // so centre on the minor axis (dmin_col = 0) and vary only dmaj.
  const double dmin_col_final = (std::abs(minor_ax.y()) > 0.1)
    ? (col_y - cy) / minor_ax.y()
    : 0.0;

  auto point_at = [&](double dmaj) -> std::pair<double, double> {
    double wx = cx + dmaj * major_ax.x() + dmin_col_final * minor_ax.x();
    double wy = cy + dmaj * major_ax.y() + dmin_col_final * minor_ax.y();
    return {wx, wy};
  };

  // Quick-fold keypoints:
  //   cloth_fold_pa — anchor (mid-length on fold column)
  //   cloth_fold_pb — neck/shoulder: FARTHER from robot (larger world X in base_footprint,
  //                   since robot faces +Y and the shirt lies along X)
  //   cloth_fold_pc — hem: CLOSER to robot (smaller world X)
  //
  // Find PB/PC from actual X extent of points near col_y (column-local scan).
  // This gives the true shirt edge along X regardless of the global percentile.
  // Band width scales with shirt width so it adapts to shirt size automatically.
  const double shirt_width_full = world_y_max_mid - world_y_min_mid;
  const double col_y_band_raw = shirt_width_full * params_.col_band_fraction;
  const double col_y_band = col_y_band_raw < 0.03 ? 0.03 : col_y_band_raw;
  double col_x_max = -std::numeric_limits<double>::max();
  double col_x_min =  std::numeric_limits<double>::max();
  std::vector<double> col_x_vals;
  col_x_vals.reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    if (std::abs(pt.y - col_y) <= col_y_band) {
      col_x_vals.push_back(pt.x);
      if (pt.x > col_x_max) col_x_max = pt.x;
      if (pt.x < col_x_min) col_x_min = pt.x;
    }
  }

  double pb_x, pc_x;
  if (!col_x_vals.empty()) {
    std::sort(col_x_vals.begin(), col_x_vals.end());
    const double pct_lo2 = params_.edge_percentile / 100.0;
    const double pct_hi2 = 1.0 - pct_lo2;
    pc_x = col_x_vals[static_cast<size_t>(pct_lo2 * (col_x_vals.size() - 1))];
    pb_x = col_x_vals[static_cast<size_t>(pct_hi2 * (col_x_vals.size() - 1))];
  } else {
    // Fallback to global percentile
    auto [p1x, p1y] = point_at(pmaj_far_dense);
    auto [p2x, p2y] = point_at(pmaj_near_dense);
    pb_x = (p1x >= p2x) ? p1x : p2x;
    pc_x = (p1x >= p2x) ? p2x : p1x;
  }

  auto [ax, ay] = point_at(pmaj_mid);
  const double bx = pb_x; const double by = col_y;
  const double cx2 = pc_x; const double cy2 = col_y;

  const double z_grasp = box.center.position.z + box.size.z / 2.0 + params_.z_grasp_offset;

  rclcpp::Time stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  broadcastPoint("cloth_fold_pa", ax,  ay,  z_grasp, stamp);
  broadcastPoint("cloth_fold_pb", bx,  by,  z_grasp, stamp);
  broadcastPoint("cloth_fold_pc", cx2, cy2, z_grasp, stamp);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Cloth OBB: cx=%.3f cy=%.3f yaw=%.2f rad  "
    "PA(anchor)=(%.3f,%.3f) PB(top)=(%.3f,%.3f) PC(hem)=(%.3f,%.3f)  z=%.3f",
    cx, cy, yaw, ax, ay, bx, by, cx2, cy2, z_grasp);

  // Debug: republish the cloud with the input frame so it is visible in RViz
  if (pub_debug_cloud_->is_activated()) {
    pub_debug_cloud_->publish(*msg);
  }
}

void ClothKeypointDetectorComponent::broadcastPoint(
  const std::string & frame_id, double x, double y, double z,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = stamp;
  t.header.frame_id = params_.base_frame;
  t.child_frame_id  = frame_id;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  // 90° rotation around Y axis: X axis points down (-Z in base_footprint).
  // q = (0, sin(π/4), 0, cos(π/4))
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.7071067811865476;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 0.7071067811865476;
  tf_broadcaster_->sendTransform(t);
}

}  // namespace pcl_object_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::ClothKeypointDetectorComponent)
