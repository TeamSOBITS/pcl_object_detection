#include "pcl_object_detection/shelf_detection_component.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace pcl_object_detection {

ShelfDetectionComponent::ShelfDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("shelf_detection", options) {
  this->declare_parameter<std::string>("input_topic", "cloud_filtered");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  
  this->declare_parameter<double>("shelf_x_min", 0.5);
  this->declare_parameter<double>("shelf_x_max", 1.5);
  this->declare_parameter<double>("shelf_y_min", -0.4);
  this->declare_parameter<double>("shelf_y_max", 0.4);
  this->declare_parameter<double>("shelf_z_min", 0.5);
  this->declare_parameter<double>("shelf_z_max", 1.0);

  this->declare_parameter<double>("plane_dist_threshold", 0.02);
  this->declare_parameter<bool>("remove_back_wall", true);

  this->declare_parameter<double>("cluster_tolerance", 0.02);
  this->declare_parameter<int>("min_cluster_size", 50);
  this->declare_parameter<int>("max_cluster_size", 2000);
  this->declare_parameter<int>("ransac_max_iterations", 200);

  this->declare_parameter<double>("object_size_x_min", 0.02);
  this->declare_parameter<double>("object_size_x_max", 0.30);
  this->declare_parameter<double>("object_size_y_min", 0.02);
  this->declare_parameter<double>("object_size_y_max", 0.30);
  this->declare_parameter<double>("object_size_z_min", 0.02);
  this->declare_parameter<double>("object_size_z_max", 0.30);
}

ShelfDetectionComponent::CallbackReturn ShelfDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Shelf Detection...");

  // Read Parameters into Struct
  params_.base_frame = this->get_parameter("base_frame").as_string();
  
  params_.shelf_x_min = this->get_parameter("shelf_x_min").as_double();
  params_.shelf_x_max = this->get_parameter("shelf_x_max").as_double();
  params_.shelf_y_min = this->get_parameter("shelf_y_min").as_double();
  params_.shelf_y_max = this->get_parameter("shelf_y_max").as_double();
  params_.shelf_z_min = this->get_parameter("shelf_z_min").as_double();
  params_.shelf_z_max = this->get_parameter("shelf_z_max").as_double();

  params_.plane_dist_threshold = this->get_parameter("plane_dist_threshold").as_double();
  params_.remove_back_wall = this->get_parameter("remove_back_wall").as_bool();

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

  // Log Parameters
  RCLCPP_INFO(this->get_logger(), "Parameters Loaded:");
  RCLCPP_INFO(this->get_logger(), "Base Frame: %s", params_.base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Shelf Zone:");
  RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", params_.shelf_x_min, params_.shelf_x_max);
  RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", params_.shelf_y_min, params_.shelf_y_max);
  RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", params_.shelf_z_min, params_.shelf_z_max);
  RCLCPP_INFO(this->get_logger(), "Plane Distance Threshold: %f", params_.plane_dist_threshold);
  RCLCPP_INFO(this->get_logger(), "Remove Back Wall: %s", params_.remove_back_wall ? "True" : "False");
  RCLCPP_INFO(this->get_logger(), "Clustering:");
  RCLCPP_INFO(this->get_logger(), "  Tolerance: %f", params_.cluster_tolerance);
  RCLCPP_INFO(this->get_logger(), "  Min Size: %d", params_.min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "  Max Size: %d", params_.max_cluster_size);
  RCLCPP_INFO(this->get_logger(), "Max RANSAC Iterations: %d", params_.ransac_max_iterations);
  RCLCPP_INFO(this->get_logger(), "Object Sizes:");
  RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", params_.obj_x_min, params_.obj_x_max);
  RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", params_.obj_y_min, params_.obj_y_max);
  RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", params_.obj_z_min, params_.obj_z_max);

  // Allocate PCL memory
  cloud_filtered_ = std::make_shared<PointCloud>();
  cloud_objects_ = std::make_shared<PointCloud>();
  cloud_shelf_zone_ = std::make_shared<PointCloud>();
  tree_ = std::make_shared<pcl::search::KdTree<PointT>>();

  // Configure PCL Defaults
  seg_.setOptimizeCoefficients(true);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(params_.ransac_max_iterations);
  ec_.setSearchMethod(tree_);

  // Create Publishers and TF Broadcaster
  auto qos = rclcpp::SensorDataQoS();
  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("shelf_objects", 10);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("shelf_debug_cloud", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

ShelfDetectionComponent::CallbackReturn ShelfDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Shelf Detection...");

  pub_detections_->on_activate();
  pub_debug_cloud_->on_activate();

  auto qos = rclcpp::SensorDataQoS();
  std::string input_topic = this->get_parameter("input_topic").as_string();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_filtered_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    std::bind(&ShelfDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

ShelfDetectionComponent::CallbackReturn ShelfDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Shelf Detection...");

  pub_detections_->on_deactivate();
  pub_debug_cloud_->on_deactivate();
  sub_filtered_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

ShelfDetectionComponent::CallbackReturn ShelfDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Shelf Detection...");

  pub_detections_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();

  cloud_filtered_.reset();
  cloud_objects_.reset();
  cloud_shelf_zone_.reset();
  tree_.reset();

  return CallbackReturn::SUCCESS;
}

ShelfDetectionComponent::CallbackReturn ShelfDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

void ShelfDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  cloud_filtered_->clear();
  cloud_shelf_zone_->clear();
  cloud_objects_->clear();

  pcl::fromROSMsg(*msg, *cloud_filtered_);
  if (cloud_filtered_->empty()) return;

  // Isolate the specific shelf bin (3D volume)
  PointCloudUtility::applyPassThrough(cloud_filtered_, cloud_shelf_zone_, "x", params_.shelf_x_min, params_.shelf_x_max);
  PointCloudUtility::applyPassThrough(cloud_shelf_zone_, cloud_shelf_zone_, "y", params_.shelf_y_min, params_.shelf_y_max);
  PointCloudUtility::applyPassThrough(cloud_shelf_zone_, cloud_shelf_zone_, "z", params_.shelf_z_min, params_.shelf_z_max);

  if (cloud_shelf_zone_->empty()) return;

  // Remove Horizontal Shelf Board (Base of the shelf)
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setDistanceThreshold(params_.plane_dist_threshold);
  seg_.setInputCloud(cloud_shelf_zone_);
  seg_.segment(*inliers, *coeffs);

  if (!inliers->indices.empty()) {
    extract_.setInputCloud(cloud_shelf_zone_);
    extract_.setIndices(inliers);
    extract_.setNegative(true);
    extract_.filter(*cloud_objects_);
  } else {
    *cloud_objects_ = *cloud_shelf_zone_;
  }

  // Remove Vertical Back Wall
  if (params_.remove_back_wall && !cloud_objects_->empty()) {
    // Look for a plane perpendicular to the X-axis (assuming robot faces the shelf straight-on)
    seg_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg_.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0)); 
    seg_.setEpsAngle(10.0 * (M_PI / 180.0)); // Allow 10 degrees of skew
    
    seg_.setInputCloud(cloud_objects_);
    seg_.segment(*inliers, *coeffs);
    if (!inliers->indices.empty()) {
        extract_.setInputCloud(cloud_objects_);
        extract_.setIndices(inliers);
        extract_.setNegative(true);
        extract_.filter(*cloud_objects_);
    }
  }

  if (cloud_objects_->empty()) return;

  // Euclidean Clustering
  std::vector<pcl::PointIndices> clusters;
  ec_.setClusterTolerance(params_.cluster_tolerance);
  ec_.setMinClusterSize(params_.min_cluster_size);
  ec_.setMaxClusterSize(params_.max_cluster_size);
  ec_.setInputCloud(cloud_objects_);
  ec_.extract(clusters);

  detection_msg_.detections.clear();
  detection_msg_.header = msg->header;

  for (const auto & cluster : clusters) {
    // Compute PCA and Bounding Box
    auto box = PointCloudUtility::computePCAAlignedBox(cloud_objects_, cluster);

    // Size Filtering
    if (box.size.x < params_.obj_x_min || box.size.x > params_.obj_x_max ||
        box.size.y < params_.obj_y_min || box.size.y > params_.obj_y_max ||
        box.size.z < params_.obj_z_min || box.size.z > params_.obj_z_max) {
      continue;
    }

    vision_msgs::msg::Detection3D det;
    det.header = msg->header;
    det.bbox = box;
    
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.pose.pose = box.center;
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
    std::string id = "shelf_obj_" + std::to_string(i);  // TODO: add robot name prefix if needed
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

} // namespace pcl_object_detection

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::ShelfDetectionComponent)