#include "pcl_object_detection/line_detection_component.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace pcl_object_detection {

LineDetectionComponent::LineDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("line_detection", options) {
  this->declare_parameter<std::string>("input_topic", "/scan");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("passthrough_axis", "y");
  this->declare_parameter<double>("passthrough_min", -1.0);
  this->declare_parameter<double>("passthrough_max", 1.0);
  this->declare_parameter<double>("distance_threshold", 0.02);
  this->declare_parameter<double>("probability", 0.95);
  this->declare_parameter<int>("ransac_max_iterations", 1000);
  this->declare_parameter<std::string>("scan_reliability", "best_effort");
  this->declare_parameter<std::string>("line_cloud_pub_reliability", "best_effort");
  this->declare_parameter<std::string>("line_pose_pub_reliability", "reliable");
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Line Detection...");

  // Read Parameters into Struct
  params_.base_frame = this->get_parameter("base_frame").as_string();
  params_.passthrough_axis = this->get_parameter("passthrough_axis").as_string();
  params_.passthrough_min = this->get_parameter("passthrough_min").as_double();
  params_.passthrough_max = this->get_parameter("passthrough_max").as_double();
  params_.distance_threshold = this->get_parameter("distance_threshold").as_double();
  params_.probability = this->get_parameter("probability").as_double();
  params_.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();
  scan_reliability_ = this->get_parameter("scan_reliability").as_string();
  line_cloud_pub_reliability_ = this->get_parameter("line_cloud_pub_reliability").as_string();
  line_pose_pub_reliability_ = this->get_parameter("line_pose_pub_reliability").as_string();

  // Log Parameters
  RCLCPP_INFO(this->get_logger(), "Parameters Loaded:");
  RCLCPP_INFO(this->get_logger(), "Base Frame: %s", params_.base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Passthrough:");
  RCLCPP_INFO(this->get_logger(), "  Axis: %s", params_.passthrough_axis.c_str());
  RCLCPP_INFO(this->get_logger(), "  Min: %f", params_.passthrough_min);
  RCLCPP_INFO(this->get_logger(), "  Max: %f", params_.passthrough_max);
  RCLCPP_INFO(this->get_logger(), "Distance Threshold: %f", params_.distance_threshold);
  RCLCPP_INFO(this->get_logger(), "Probability: %f", params_.probability);
  RCLCPP_INFO(this->get_logger(), "Max RANSAC Iterations: %d", params_.ransac_max_iterations);
  RCLCPP_INFO(this->get_logger(), "QoS Reliability:");
  RCLCPP_INFO(this->get_logger(), "  scan: %s", scan_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  line_cloud pub: %s", line_cloud_pub_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  line_pose pub: %s", line_pose_pub_reliability_.c_str());

  // Allocate PCL Memory
  cloud_raw_ = std::make_shared<PointCloud>();
  cloud_line_ = std::make_shared<PointCloud>();

  // Configure PCL Defaults
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(params_.ransac_max_iterations);

  // Create Lifecycle Publishers
  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable"
      ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };
  pub_line_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "line_cloud", make_qos(line_cloud_pub_reliability_, 10));
  pub_line_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "line_pose", make_qos(line_pose_pub_reliability_, 10));

  return CallbackReturn::SUCCESS;
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Line Detection...");

  // Activate Publishers
  pub_line_cloud_->on_activate();
  pub_line_pose_->on_activate();

  // Start Data Flow via Subscription
  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable"
      ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };
  std::string input_topic = this->get_parameter("input_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "scan QoS reliability: %s", scan_reliability_.c_str());

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  // Explicit callback group so the executor reliably services this
  // subscription (created during a lifecycle transition in a shared container).
  if (!cb_group_) {
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  sub_options.callback_group = cb_group_;

  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic, make_qos(scan_reliability_, 10),
    std::bind(&LineDetectionComponent::scanCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Line Detection...");

  // Deactivate Publishers
  pub_line_cloud_->on_deactivate();
  pub_line_pose_->on_deactivate();
  
  // Halt data flow
  sub_scan_.reset();

  return CallbackReturn::SUCCESS;
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Line Detection...");

  // Release all heap-allocated objects back to the system
  pub_line_cloud_.reset();
  pub_line_pose_.reset();

  cloud_raw_.reset();
  cloud_line_.reset();

  return CallbackReturn::SUCCESS;
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Shutting down Line Detection...");
  return CallbackReturn::SUCCESS;
}

void LineDetectionComponent::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // Reset PCL buffers
  cloud_raw_->clear();
  cloud_line_->clear();

  // Project 2D Scan to 3D PointCloud
  sensor_msgs::msg::PointCloud2 pc2_msg;
  try {
    projector_.projectLaser(*msg, pc2_msg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Scan projection failed: %s", e.what());
    return;
  }
  pcl::fromROSMsg(pc2_msg, *cloud_raw_);

  if (cloud_raw_->empty()) return;

  // Filter area of interest
  PointCloudUtility::applyPassThrough(
    cloud_raw_, cloud_raw_, params_.passthrough_axis, 
    params_.passthrough_min, params_.passthrough_max);

  if (cloud_raw_->empty()) return;

  // RANSAC Line Fitting
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  
  seg_.setDistanceThreshold(params_.distance_threshold);
  seg_.setProbability(params_.probability);
  seg_.setInputCloud(cloud_raw_);
  seg_.segment(*inliers, *coeffs);

  if (inliers->indices.empty()) return;

  // Extract Line Points
  extract_.setInputCloud(cloud_raw_);
  extract_.setIndices(inliers);
  extract_.setNegative(false);
  extract_.filter(*cloud_line_);

  // Calculate Geometry
  Eigen::Vector3f point_on_line(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
  Eigen::Vector3f line_dir(coeffs->values[3], coeffs->values[4], coeffs->values[5]);

  double yaw = std::atan2(line_dir.y(), line_dir.x());

  // Create Pose message
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = msg->header;
  
  pose_msg.pose.position.x = point_on_line.x();
  pose_msg.pose.position.y = point_on_line.y();
  pose_msg.pose.position.z = point_on_line.z();
  pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
  pose_msg.pose.orientation.w = std::cos(yaw / 2.0);

  pub_line_pose_->publish(pose_msg);

  // Debug Cloud Publishing
  if (pub_line_cloud_->get_subscription_count() > 0) {
    auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_line_, *output_msg);
    output_msg->header = msg->header;
    pub_line_cloud_->publish(std::move(output_msg));
  }
}

}  // namespace pcl_object_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::LineDetectionComponent)
