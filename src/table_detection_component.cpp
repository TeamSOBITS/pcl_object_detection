#include "pcl_object_detection/table_detection_component.hpp"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_object_detection {

TableDetectionComponent::TableDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("table_detection", options) {
  this->declare_parameter<std::string>("input_topic", "filtered_cloud");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<double>("table_height_min", 0.5);
  this->declare_parameter<double>("table_height_max", 1.2);
  this->declare_parameter<double>("plane_dist_threshold", 0.02);
  this->declare_parameter<double>("cluster_tolerance", 0.03);
  this->declare_parameter<int>("min_cluster_size", 100);
  this->declare_parameter<int>("max_cluster_size", 5000);
  this->declare_parameter<int>("ransac_max_iterations", 200);

  this->declare_parameter<double>("object_size_x_min", 0.02);
  this->declare_parameter<double>("object_size_x_max", 0.30);
  this->declare_parameter<double>("object_size_y_min", 0.02);
  this->declare_parameter<double>("object_size_y_max", 0.30);
  this->declare_parameter<double>("object_size_z_min", 0.02);
  this->declare_parameter<double>("object_size_z_max", 0.40);

  this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  this->declare_parameter<std::string>("detections_pub_reliability", "reliable");
  this->declare_parameter<std::string>("debug_pub_reliability", "best_effort");
}

TableDetectionComponent::CallbackReturn TableDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Table Detection...");

  // Read Parameters into Struct
  params_.base_frame = this->get_parameter("base_frame").as_string();
  params_.table_height_min = this->get_parameter("table_height_min").as_double();
  params_.table_height_max = this->get_parameter("table_height_max").as_double();
  params_.plane_dist_threshold = this->get_parameter("plane_dist_threshold").as_double();
  params_.cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
  params_.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
  params_.max_cluster_size = this->get_parameter("max_cluster_size").as_int();
  params_.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();

  params_.obj_x_min = this->get_parameter("object_size_x_min").as_double();
  params_.obj_x_max = this->get_parameter("object_size_x_max").as_double();
  params_.obj_y_min = this->get_parameter("object_size_y_min").as_double();
  params_.obj_y_max = this->get_parameter("object_size_y_max").as_double();
  params_.obj_z_min = this->get_parameter("object_size_z_min").as_double();
  params_.obj_z_max = this->get_parameter("object_size_z_max").as_double();

  cloud_reliability_ = this->get_parameter("cloud_reliability").as_string();
  detections_pub_reliability_ = this->get_parameter("detections_pub_reliability").as_string();
  debug_pub_reliability_ = this->get_parameter("debug_pub_reliability").as_string();

  // Log Parameters
  RCLCPP_INFO(this->get_logger(), "Parameters Loaded:");
  RCLCPP_INFO(this->get_logger(), "Base Frame: %s", params_.base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Table Height Min: %f", params_.table_height_min);
  RCLCPP_INFO(this->get_logger(), "Table Height Max: %f", params_.table_height_max);
  RCLCPP_INFO(this->get_logger(), "Plane Distance Threshold: %f", params_.plane_dist_threshold);
  RCLCPP_INFO(this->get_logger(), "Clustering:");
  RCLCPP_INFO(this->get_logger(), "  Tolerance: %f", params_.cluster_tolerance);
  RCLCPP_INFO(this->get_logger(), "  Min Size: %d", params_.min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "  Max Size: %d", params_.max_cluster_size);
  RCLCPP_INFO(this->get_logger(), "Max RANSAC Iterations: %d", params_.ransac_max_iterations);
  RCLCPP_INFO(this->get_logger(), "Object Size Constraints:");
  RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", params_.obj_x_min, params_.obj_x_max);
  RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", params_.obj_y_min, params_.obj_y_max);
  RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", params_.obj_z_min, params_.obj_z_max);

  // Allocate PCL memory
  cloud_filtered_ = std::make_shared<PointCloud>();
  cloud_objects_ = std::make_shared<PointCloud>();
  cloud_table_zone_ = std::make_shared<PointCloud>();
  tree_ = std::make_shared<pcl::search::KdTree<PointT>>();

  // Configure PCL Defaults
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(params_.ransac_max_iterations);
  ec_.setSearchMethod(tree_);

  // Create Publishers and TF Broadcaster
  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };

  RCLCPP_INFO(this->get_logger(), "Detections Pub Reliability: %s", detections_pub_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug Pub Reliability: %s", debug_pub_reliability_.c_str());

  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    "table_objects", make_qos(detections_pub_reliability_, 10));
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "table_debug_cloud", make_qos(debug_pub_reliability_, 10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

TableDetectionComponent::CallbackReturn TableDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Table Detection...");

  // Activate Publishers
  pub_detections_->on_activate();
  pub_debug_cloud_->on_activate();

  // Start Data Flow via Subscription
  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };
  std::string input_topic = this->get_parameter("input_topic").as_string();

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_filtered_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, make_qos(cloud_reliability_, 10),
    std::bind(&TableDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

TableDetectionComponent::CallbackReturn TableDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Table Detection...");

  // Deactivate Publishers
  pub_detections_->on_deactivate();
  pub_debug_cloud_->on_deactivate();

  // Halt Data Flow
  sub_filtered_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

TableDetectionComponent::CallbackReturn TableDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Table Detection...");

  // Release all heap-allocated objects back to the system
  pub_detections_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();
  
  cloud_filtered_.reset();
  cloud_objects_.reset();
  cloud_table_zone_.reset();
  tree_.reset();

  return CallbackReturn::SUCCESS;
}

TableDetectionComponent::CallbackReturn TableDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Shutting down Table Detection...");
  return CallbackReturn::SUCCESS;
}

void TableDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Reset PCL buffers
  cloud_filtered_->clear();
  cloud_objects_->clear();
  cloud_table_zone_->clear();

  pcl::fromROSMsg(*msg, *cloud_filtered_);
  if (cloud_filtered_->empty()) return;

  // Isolate the height range where the table is expected
  PointCloudUtility::applyPassThrough(
    cloud_filtered_, cloud_table_zone_, "z", 
    params_.table_height_min, params_.table_height_max);

  if (cloud_table_zone_->empty()) return;

  // Plane Segmentation (Find Table Surface)
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  
  seg_.setDistanceThreshold(params_.plane_dist_threshold);
  seg_.setInputCloud(cloud_table_zone_);
  seg_.segment(*inliers, *coeffs);

  if (inliers->indices.empty()) return;

  // Extract Objects ABOVE the table
  extract_.setInputCloud(cloud_table_zone_);
  extract_.setIndices(inliers);
  extract_.setNegative(true);
  extract_.filter(*cloud_objects_);

  if (cloud_objects_->empty()) return;

  // Euclidean Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  ec_.setClusterTolerance(params_.cluster_tolerance);
  ec_.setMinClusterSize(params_.min_cluster_size);
  ec_.setMaxClusterSize(params_.max_cluster_size);
  ec_.setInputCloud(cloud_objects_);
  ec_.extract(cluster_indices);

  detection_msg_.detections.clear();
  detection_msg_.header = msg->header;

  for (const auto & cluster : cluster_indices) {
    // Compute PCA and Bounding Box
    auto box = PointCloudUtility::computePCAAlignedBox(cloud_objects_, cluster);

    // Size Filtering
    if (box.size.x < params_.obj_x_min || box.size.x > params_.obj_x_max ||
        box.size.y < params_.obj_y_min || box.size.y > params_.obj_y_max ||
        box.size.z < params_.obj_z_min || box.size.z > params_.obj_z_max) {
      continue;
    }

    // Create Detection Message
    vision_msgs::msg::Detection3D det;
    det.header = msg->header;
    det.bbox = box;
    
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.pose.pose = det.bbox.center;
    hyp.hypothesis.score = 1.0;
    det.results.push_back(hyp);

    detection_msg_.detections.push_back(det);
  }

  // Distance Sorting
  std::sort(detection_msg_.detections.begin(), detection_msg_.detections.end(),
    [](const auto& a, const auto& b) {
      return std::hypot(a.bbox.center.position.x, a.bbox.center.position.y) < 
             std::hypot(b.bbox.center.position.x, b.bbox.center.position.y);
    });

  // ID Assignment and TF Broadcast
  for (size_t i = 0; i < detection_msg_.detections.size(); ++i) {
    std::string id = "table_obj_" + std::to_string(i);  // TODO: add robot name prefix if needed
    detection_msg_.detections[i].id = id;

    geometry_msgs::msg::TransformStamped t;
    t.header = detection_msg_.header;
    t.child_frame_id = id;
    t.transform.translation.x = detection_msg_.detections[i].bbox.center.position.x;
    t.transform.translation.y = detection_msg_.detections[i].bbox.center.position.y;
    t.transform.translation.z = detection_msg_.detections[i].bbox.center.position.z;
    t.transform.rotation = detection_msg_.detections[i].bbox.center.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  pub_detections_->publish(detection_msg_);

  // Debug Cloud Publishing
  if (pub_debug_cloud_->get_subscription_count() > 0) {
    auto debug_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_objects_, *debug_msg);
    debug_msg->header = msg->header;
    pub_debug_cloud_->publish(std::move(debug_msg));
  }
}

}  // namespace pcl_object_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::TableDetectionComponent)
