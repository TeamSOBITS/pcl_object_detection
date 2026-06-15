#include "pcl_object_detection/floor_detection_component.hpp"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_object_detection {

FloorDetectionComponent::FloorDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("floor_detection", options) {
  this->declare_parameter<std::string>("input_topic", "filtered_cloud");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  
  this->declare_parameter<double>("floor_height_min", -0.1);
  this->declare_parameter<double>("floor_height_max", 0.4);
  this->declare_parameter<double>("plane_dist_threshold", 0.03);
  this->declare_parameter<double>("cluster_tolerance", 0.05);
  this->declare_parameter<int>("min_cluster_size", 100);
  this->declare_parameter<int>("max_cluster_size", 10000);
  this->declare_parameter<int>("ransac_max_iterations", 200);

  this->declare_parameter<bool>("use_vertical_filter", true);
  this->declare_parameter<double>("vertical_slice_thickness", 0.02);
  this->declare_parameter<double>("vertical_xy_cell_size", 0.03);
  this->declare_parameter<double>("vertical_required_continuity", 0.15);
  this->declare_parameter<int>("vertical_min_pts_per_cell", 3);

  this->declare_parameter<double>("object_size_x_min", 0.05);
  this->declare_parameter<double>("object_size_x_max", 0.80);
  this->declare_parameter<double>("object_size_y_min", 0.05);
  this->declare_parameter<double>("object_size_y_max", 0.80);
  this->declare_parameter<double>("object_size_z_min", 0.05);
  this->declare_parameter<double>("object_size_z_max", 0.80);

  this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  this->declare_parameter<std::string>("detections_pub_reliability", "reliable");
  this->declare_parameter<std::string>("debug_pub_reliability", "best_effort");
}

FloorDetectionComponent::CallbackReturn FloorDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Floor Detection...");

  // Read Parameters into Struct
  params_.base_frame = this->get_parameter("base_frame").as_string();
  params_.floor_height_min = this->get_parameter("floor_height_min").as_double();
  params_.floor_height_max = this->get_parameter("floor_height_max").as_double();
  params_.plane_dist_threshold = this->get_parameter("plane_dist_threshold").as_double();
  params_.cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
  params_.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
  params_.max_cluster_size = this->get_parameter("max_cluster_size").as_int();
  params_.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();

  params_.use_vertical_filter = this->get_parameter("use_vertical_filter").as_bool();
  params_.slice_thickness = this->get_parameter("vertical_slice_thickness").as_double();
  params_.xy_cell_size = this->get_parameter("vertical_xy_cell_size").as_double();
  params_.required_continuity = this->get_parameter("vertical_required_continuity").as_double();
  params_.min_pts_per_cell = this->get_parameter("vertical_min_pts_per_cell").as_int();

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
  RCLCPP_INFO(this->get_logger(), "Floor Height Min: %f", params_.floor_height_min);
  RCLCPP_INFO(this->get_logger(), "Floor Height Max: %f", params_.floor_height_max);
  RCLCPP_INFO(this->get_logger(), "Plane Distance Threshold: %f", params_.plane_dist_threshold);
  RCLCPP_INFO(this->get_logger(), "Clustering:");
  RCLCPP_INFO(this->get_logger(), "  Tolerance: %f", params_.cluster_tolerance);
  RCLCPP_INFO(this->get_logger(), "  Min Size: %d", params_.min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "  Max Size: %d", params_.max_cluster_size);
  RCLCPP_INFO(this->get_logger(), "Max RANSAC Iterations: %d", params_.ransac_max_iterations);

  RCLCPP_INFO(this->get_logger(), "Vertical Structure Filter:");
  RCLCPP_INFO(this->get_logger(), "Use Vertical Filter: %s", params_.use_vertical_filter ? "True" : "False");
  RCLCPP_INFO(this->get_logger(), "Slice Thickness: %f", params_.slice_thickness);
  RCLCPP_INFO(this->get_logger(), "XY Cell Size: %f", params_.xy_cell_size);
  RCLCPP_INFO(this->get_logger(), "Required Continuity: %f", params_.required_continuity);
  RCLCPP_INFO(this->get_logger(), "Min Points Per Cell: %d", params_.min_pts_per_cell);

  RCLCPP_INFO(this->get_logger(), "Object Size Constraints:");
  RCLCPP_INFO(this->get_logger(), "X: [%f, %f]", params_.obj_x_min, params_.obj_x_max);
  RCLCPP_INFO(this->get_logger(), "Y: [%f, %f]", params_.obj_y_min, params_.obj_y_max);
  RCLCPP_INFO(this->get_logger(), "Z: [%f, %f]", params_.obj_z_min, params_.obj_z_max);

  RCLCPP_INFO(this->get_logger(), "QoS Reliability:");
  RCLCPP_INFO(this->get_logger(), "  Cloud Subscription: %s", cloud_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Detections Publisher: %s", detections_pub_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Debug Publisher: %s", debug_pub_reliability_.c_str());

  // Allocate PCL Memory
  cloud_filtered_ = std::make_shared<PointCloud>();
  cloud_objects_ = std::make_shared<PointCloud>();
  cloud_floor_zone_ = std::make_shared<PointCloud>();
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
  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    "floor_objects", make_qos(detections_pub_reliability_, 10));
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "floor_debug_cloud", make_qos(debug_pub_reliability_, 10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

FloorDetectionComponent::CallbackReturn FloorDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Floor Detection...");

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
    std::bind(&FloorDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

FloorDetectionComponent::CallbackReturn FloorDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Floor Detection...");

  // Deactivate Publishers
  pub_detections_->on_deactivate();
  pub_debug_cloud_->on_deactivate();
  
  // Halt Data Flow
  sub_filtered_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

FloorDetectionComponent::CallbackReturn FloorDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Floor Detection...");

  // Release all heap-allocated objects back to the system
  pub_detections_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();

  cloud_filtered_.reset();
  cloud_objects_.reset();
  cloud_floor_zone_.reset();
  tree_.reset();

  return CallbackReturn::SUCCESS;
}

FloorDetectionComponent::CallbackReturn FloorDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Shutting down Floor Detection...");
  return CallbackReturn::SUCCESS;
}

bool FloorDetectionComponent::isVerticalStructure(const PointCloud::Ptr& cloud, const pcl::PointIndices& cluster) {
  float min_x = std::numeric_limits<float>::max(), max_x = -min_x;
  float min_y = min_x, max_y = max_x;
  float max_z = -std::numeric_limits<float>::max();

  for (const auto& idx : cluster.indices) {
    const auto& pt = cloud->points[idx];
    min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
    max_z = std::max(max_z, pt.z);
  }

  // If the cluster is very short, it's not a pillar
  if (max_z < params_.required_continuity) return false;

  int grid_w = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / params_.xy_cell_size)));
  int grid_h = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / params_.xy_cell_size)));
  int num_slices = static_cast<int>(std::ceil(max_z / params_.slice_thickness));

  // Protection against massive clusters (e.g. walls)
  if (grid_w * grid_h > 2500) return true; 

  std::vector<int> grid_counts(grid_w * grid_h * num_slices, 0);
  for (const auto& idx : cluster.indices) {
    const auto& pt = cloud->points[idx];
    int s_idx = std::floor(pt.z / params_.slice_thickness);
    int x_idx = std::floor((pt.x - min_x) / params_.xy_cell_size);
    int y_idx = std::floor((pt.y - min_y) / params_.xy_cell_size);
    
    if (s_idx >= 0 && s_idx < num_slices && x_idx < grid_w && y_idx < grid_h) {
      grid_counts[(x_idx * grid_h + y_idx) * num_slices + s_idx]++;
    }
  }

  for (int x = 0; x < grid_w; ++x) {
    for (int y = 0; y < grid_h; ++y) {
      double continuous_h = 0.0;
      int base = (x * grid_h + y) * num_slices;
      for (int s = 0; s < num_slices; ++s) {
        if (grid_counts[base + s] >= params_.min_pts_per_cell) {
          continuous_h += params_.slice_thickness;
          if (continuous_h >= params_.required_continuity) return true; 
        } else {
          continuous_h = 0.0;
        }
      }
    }
  }
  return false;
}

void FloorDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Reset PCL buffers
  cloud_filtered_->clear();
  cloud_objects_->clear();
  cloud_floor_zone_->clear();

  pcl::fromROSMsg(*msg, *cloud_filtered_);
  if (cloud_filtered_->empty()) return;

  // Isolate the floor area
  PointCloudUtility::applyPassThrough(
    cloud_filtered_, cloud_floor_zone_, "z", 
    params_.floor_height_min, params_.floor_height_max);
  if (cloud_floor_zone_->empty()) return;

  // Remove Ground Plane
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  seg_.setDistanceThreshold(params_.plane_dist_threshold);
  seg_.setInputCloud(cloud_floor_zone_);
  seg_.segment(*inliers, *coeffs);

  if (inliers->indices.empty()) return;

  // Extract Objects ABOVE the Floor
  extract_.setInputCloud(cloud_floor_zone_);
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
    // Vertical Structure Filtering (Ignore table legs, pillars)
    if (params_.use_vertical_filter && isVerticalStructure(cloud_objects_, cluster)) {
      continue;
    }

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
    std::string id = "floor_obj_" + std::to_string(i);  // TODO: add robot name prefix if needed
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
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::FloorDetectionComponent)
