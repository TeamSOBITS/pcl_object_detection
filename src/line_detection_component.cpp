#include "pcl_object_detection/line_detection_component.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace pcl_object_detection {

LineDetectionComponent::LineDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("line_detection", options){
  this->declare_parameter<std::string>("input_topic", "/scan");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("passthrough_axis", "y");
  this->declare_parameter<double>("passthrough_min", -1.0);
  this->declare_parameter<double>("passthrough_max", 1.0);
  this->declare_parameter<double>("distance_threshold", 0.02);
  this->declare_parameter<double>("probability", 0.95);
  this->declare_parameter<int>("ransac_max_iterations", 1000);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  }

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Line Detection...");

  // Read Parameters into Struct
  params_.odom_frame = this->get_parameter("odom_frame").as_string();
  params_.base_frame = this->get_parameter("base_frame").as_string();
  params_.passthrough_axis = this->get_parameter("passthrough_axis").as_string();
  params_.passthrough_min = this->get_parameter("passthrough_min").as_double();
  params_.passthrough_max = this->get_parameter("passthrough_max").as_double();
  params_.distance_threshold = this->get_parameter("distance_threshold").as_double();
  params_.probability = this->get_parameter("probability").as_double();
  params_.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();

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

  // Allocate PCL Memory
  cloud_raw_ = std::make_shared<PointCloud>();
  cloud_line_ = std::make_shared<PointCloud>();

  // Configure PCL Defaults
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(params_.ransac_max_iterations);

  // Create Lifecycle Publishers
  auto qos = rclcpp::SensorDataQoS().best_effort();
  pub_line_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_cloud", qos);
  pub_line_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("line_pose", 10);
  dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  return CallbackReturn::SUCCESS;
}

LineDetectionComponent::CallbackReturn LineDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Line Detection...");

  // Activate Publishers
  pub_line_cloud_->on_activate();
  pub_line_pose_->on_activate();

  // Start Data Flow via Subscription
  auto qos = rclcpp::SensorDataQoS();
  std::string input_topic = this->get_parameter("input_topic").as_string();

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic, qos,
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
  //=========
  PointCloud::Ptr cloud_in_odom(new PointCloud);

  geometry_msgs::msg::TransformStamped tf_to_odom;

  try{
      tf_to_odom = tfBuffer_->lookupTransform(
        params_.odom_frame,     // target
        msg->header.frame_id,   // source
        tf2::TimePointZero,
        tf2::durationFromSec(0.2)
      );
    //Transform point cloud from laser frame to odom frame
    pcl_ros::transformPointCloud(*cloud_raw_, *cloud_in_odom, tf_to_odom);

    // Filter area of interest
    PointCloudUtility::applyPassThrough(
      cloud_in_odom, cloud_in_odom, params_.passthrough_axis, 
      params_.passthrough_min, params_.passthrough_max);

    } catch (tf2::TransformException& ex) { return; }
  
  if (cloud_in_odom->empty()) return;

  // Transform point cloud from odom frame to base frame
  PointCloud::Ptr cloud_in_base(new PointCloud);

  try {
    geometry_msgs::msg::TransformStamped tf_base;
    tf_base = tfBuffer_->lookupTransform(params_.base_frame, params_.odom_frame, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*cloud_in_odom, *cloud_in_base, tf_base);
  } catch (tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "[bag_handle_estimator] Failed to transform point cloud: %s", ex.what());
      // ROS_WARN("[bag_handle_estimator] Failed to transform point cloud: %s", ex.what());
      return;
  }
  //=========


  if (cloud_in_base->empty()) return;

  // RANSAC Line Fitting
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  

  Eigen::Vector3f axis = Eigen::Vector3f(1.0, 0.0, 0.0);
  seg_.setAxis(axis);
  seg_.setEpsAngle(0.174533);
  seg_.setDistanceThreshold(params_.distance_threshold);
  seg_.setProbability(params_.probability);
  seg_.setInputCloud(cloud_in_base);
  seg_.segment(*inliers, *coeffs);

  if (inliers->indices.empty()) return;

  // Extract Line Points
  extract_.setInputCloud(cloud_in_base);
  extract_.setIndices(inliers);
  extract_.setNegative(false);
  extract_.filter(*cloud_line_);

  // Calculate Geometry
  Eigen::Vector3f point_on_line(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
  Eigen::Vector3f line_dir(coeffs->values[3], coeffs->values[4], coeffs->values[5]);

  double yaw = std::atan2(line_dir.y(), line_dir.x());

  // Create Pose message
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = params_.base_frame;
  
  pose_msg.pose.position.x = point_on_line.x();
  pose_msg.pose.position.y = point_on_line.y();
  pose_msg.pose.position.z = point_on_line.z();
  pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
  pose_msg.pose.orientation.w = std::cos(yaw / 2.0);

  pub_line_pose_->publish(pose_msg);
  broadcast_line_tf(point_on_line, yaw, params_.base_frame);
  // Debug Cloud Publishing
  if (pub_line_cloud_->get_subscription_count() > 0) {
    auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_line_, *output_msg);
    output_msg->header = msg->header;
    output_msg->header.frame_id = params_.base_frame;
    pub_line_cloud_->publish(std::move(output_msg));
  }
}

void LineDetectionComponent::broadcast_line_tf(const Eigen::Vector3f& pos, double yaw, const std::string& frame_id) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = frame_id;
  t.child_frame_id = "bag_line";
  t.transform.translation.x = pos.x();
  t.transform.translation.y = pos.y();
  t.transform.translation.z = pos.z();
  
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  t.transform.rotation = tf2::toMsg(q);

  dynamic_broadcaster_->sendTransform(t);
}

}  // namespace pcl_object_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::LineDetectionComponent)
