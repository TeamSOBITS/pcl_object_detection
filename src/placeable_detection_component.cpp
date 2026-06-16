#include "pcl_object_detection/placeable_detection_component.hpp"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <limits>

namespace pcl_object_detection {

PlaceableDetectionComponent::PlaceableDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("placeable_detection", options) {
  this->declare_parameter<std::string>("input_topic", "filtered_cloud");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  
  this->declare_parameter<double>("place_x_min", 0.3);
  this->declare_parameter<double>("place_x_max", 1.5);
  this->declare_parameter<double>("place_y_min", -1.0);
  this->declare_parameter<double>("place_y_max", 1.0);
  this->declare_parameter<double>("place_z_min", 0.4);
  this->declare_parameter<double>("place_z_max", 1.2);

  this->declare_parameter<double>("search_interval", 0.02);
  this->declare_parameter<double>("obstacle_tolerance", 0.10);
  this->declare_parameter<double>("edge_margin", 0.05);
  this->declare_parameter<double>("plane_dist_threshold", 0.02);
  this->declare_parameter<int>("ransac_max_iterations", 200);

  this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  this->declare_parameter<std::string>("detections_pub_reliability", "reliable");
  this->declare_parameter<std::string>("debug_pub_reliability", "reliable");
}

PlaceableDetectionComponent::CallbackReturn PlaceableDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Placeable Spot Detection...");

  // Read Parameters into Struct
  params_.base_frame = this->get_parameter("base_frame").as_string();
  
  params_.place_x_min = this->get_parameter("place_x_min").as_double();
  params_.place_x_max = this->get_parameter("place_x_max").as_double();
  params_.place_y_min = this->get_parameter("place_y_min").as_double();
  params_.place_y_max = this->get_parameter("place_y_max").as_double();
  params_.place_z_min = this->get_parameter("place_z_min").as_double();
  params_.place_z_max = this->get_parameter("place_z_max").as_double();

  params_.search_interval = this->get_parameter("search_interval").as_double();
  params_.obstacle_tolerance = this->get_parameter("obstacle_tolerance").as_double();
  params_.edge_margin = this->get_parameter("edge_margin").as_double();

  params_.plane_dist_threshold = this->get_parameter("plane_dist_threshold").as_double();
  params_.ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();

  cloud_reliability_ = this->get_parameter("cloud_reliability").as_string();
  detections_pub_reliability_ = this->get_parameter("detections_pub_reliability").as_string();
  debug_pub_reliability_ = this->get_parameter("debug_pub_reliability").as_string();

  // Log Parameters
  RCLCPP_INFO(this->get_logger(), "Parameters Loaded:");
  RCLCPP_INFO(this->get_logger(), "Base Frame: %s", params_.base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Place Search Volume:");
  RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", params_.place_x_min, params_.place_x_max);
  RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", params_.place_y_min, params_.place_y_max);
  RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", params_.place_z_min, params_.place_z_max);
  RCLCPP_INFO(this->get_logger(), "Search Parameters:");
  RCLCPP_INFO(this->get_logger(), "  Search Interval: %f", params_.search_interval);
  RCLCPP_INFO(this->get_logger(), "  Obstacle Tolerance: %f", params_.obstacle_tolerance);
  RCLCPP_INFO(this->get_logger(), "  Edge Margin: %f", params_.edge_margin);
  RCLCPP_INFO(this->get_logger(), "  Plane Distance Threshold: %f", params_.plane_dist_threshold);
  RCLCPP_INFO(this->get_logger(), "  Max Iterations: %d", params_.ransac_max_iterations);
  RCLCPP_INFO(this->get_logger(), "QoS Reliability:");
  RCLCPP_INFO(this->get_logger(), "  Cloud (sub): %s", cloud_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Detections (pub): %s", detections_pub_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Debug Cloud (pub): %s", debug_pub_reliability_.c_str());

  // Allocate PCL Memory
  cloud_filtered_ = std::make_shared<PointCloud>();
  cloud_table_zone_ = std::make_shared<PointCloud>();
  cloud_plane_ = std::make_shared<PointCloud>();
  cloud_obstacles_ = std::make_shared<PointCloud>();
  cloud_placeable_ = std::make_shared<PointCloud>();
  tree_ = std::make_shared<pcl::search::KdTree<PointT>>();

  // Configure PCL Defaults
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(params_.ransac_max_iterations);

  // Create Publishers and TF Broadcaster
  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };
  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    "placeable_poses", make_qos(detections_pub_reliability_, 10));
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "placeable_debug_cloud", make_qos(debug_pub_reliability_, 10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

PlaceableDetectionComponent::CallbackReturn PlaceableDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Placeable Detection...");

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
    std::bind(&PlaceableDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

PlaceableDetectionComponent::CallbackReturn PlaceableDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Placeable Detection...");

  // Deactivate Publishers
  pub_detections_->on_deactivate();
  pub_debug_cloud_->on_deactivate();

  // Halt Data Flow
  sub_filtered_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

PlaceableDetectionComponent::CallbackReturn PlaceableDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Placeable Detection...");

  // Release all heap-allocated objects back to the system
  pub_detections_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();

  cloud_filtered_.reset();
  cloud_table_zone_.reset();
  cloud_plane_.reset();
  cloud_obstacles_.reset();
  cloud_placeable_.reset();
  tree_.reset();

  return CallbackReturn::SUCCESS;
}

PlaceableDetectionComponent::CallbackReturn PlaceableDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Shutting down Floor Detection...");
  return CallbackReturn::SUCCESS;
}

void PlaceableDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Reset PCL buffers
  cloud_filtered_->clear();
  cloud_table_zone_->clear();
  cloud_plane_->clear();
  cloud_obstacles_->clear();
  cloud_placeable_->clear();

  // The placement search assumes the cloud is already in base_frame (X forward,
  // Y left, Z up, origin at the robot). If the producer ever publishes another
  // frame, the grid search and the broadcast TF would silently be wrong.
  if (msg->header.frame_id != params_.base_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Input cloud frame '%s' != base_frame '%s'; skipping (cloud is not transformed here).",
      msg->header.frame_id.c_str(), params_.base_frame.c_str());
    return;
  }

  // Publishes the PLACEABLE region (the debug view): every grid spot that passed
  // the obstacle-clearance gate, i.e. where an object could actually be set down.
  // Called on every exit path so the topic stays alive and RViz never sees it go
  // stale — on any early return cloud_placeable_ is empty and we publish an empty
  // cloud (a valid, displayable "no placeable area this frame" message).
  auto publishDebugCloud = [&]() {
    if (pub_debug_cloud_->get_subscription_count() == 0) return;
    auto debug_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_placeable_, *debug_msg);
    debug_msg->header = msg->header;
    pub_debug_cloud_->publish(std::move(debug_msg));
  };

  pcl::fromROSMsg(*msg, *cloud_filtered_);
  if (cloud_filtered_->empty()) { publishDebugCloud(); return; }

  // Isolate the table volume
  PointCloudUtility::applyPassThrough(cloud_filtered_, cloud_table_zone_, "x", params_.place_x_min, params_.place_x_max);
  PointCloudUtility::applyPassThrough(cloud_table_zone_, cloud_table_zone_, "y", params_.place_y_min, params_.place_y_max);
  PointCloudUtility::applyPassThrough(cloud_table_zone_, cloud_table_zone_, "z", params_.place_z_min, params_.place_z_max);

  if (cloud_table_zone_->empty()) { publishDebugCloud(); return; }

  // Plane Segmentation (Find Table Surface)
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  
  seg_.setDistanceThreshold(params_.plane_dist_threshold);
  seg_.setInputCloud(cloud_table_zone_);
  seg_.segment(*inliers, *coeffs);

  if (inliers->indices.empty()) { publishDebugCloud(); return; }

  extract_.setInputCloud(cloud_table_zone_);
  extract_.setIndices(inliers);
  
  // Get Plane
  extract_.setNegative(false);
  extract_.filter(*cloud_plane_);   
  
  // Get Obstacles (Everything else)
  extract_.setNegative(true);
  extract_.filter(*cloud_obstacles_); 

  if (cloud_plane_->empty()) { publishDebugCloud(); return; }

  // Determine the Searchable Area (Table boundaries minus safety margin)
  Eigen::Vector4f min_pt, max_pt, centroid;
  pcl::getMinMax3D(*cloud_plane_, min_pt, max_pt);
  pcl::compute3DCentroid(*cloud_plane_, centroid);

  // Set up KD-Tree for Obstacle avoidance.
  // Project obstacles onto the table plane (flatten Z to the surface height) so
  // clearance is measured as a horizontal footprint distance, not a 3D distance.
  // Otherwise a tall object's upper points read as "far" even when its base
  // overlaps the candidate spot in XY.
  const float plane_z = static_cast<float>(centroid[2]);
  bool has_obstacles = !cloud_obstacles_->empty();
  if (has_obstacles) {
    auto cloud_obstacles_flat = std::make_shared<PointCloud>();
    cloud_obstacles_flat->reserve(cloud_obstacles_->size());
    for (const auto & pt : cloud_obstacles_->points) {
      PointT flat;
      flat.x = pt.x;
      flat.y = pt.y;
      flat.z = plane_z;
      cloud_obstacles_flat->push_back(flat);
    }
    tree_->setInputCloud(cloud_obstacles_flat);
  }

  // Guard against a degenerate searchable extent (plane patch smaller than the
  // margins on either axis): the loops below would silently never execute.
  const double x_lo = min_pt.x() + params_.edge_margin;
  const double x_hi = max_pt.x() - params_.edge_margin;
  const double y_lo = min_pt.y() + params_.edge_margin;
  const double y_hi = max_pt.y() - params_.edge_margin;
  if (x_lo >= x_hi || y_lo >= y_hi) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Searchable plane extent too small for edge_margin %.3f (x:[%.3f,%.3f] y:[%.3f,%.3f]).",
      params_.edge_margin, min_pt.x(), max_pt.x(), min_pt.y(), max_pt.y());
    publishDebugCloud();
    return;
  }

  // Grid Search. Among all spots that satisfy the obstacle-clearance gate,
  // pick the one CLOSEST to the robot (origin of base_frame is at the robot).
  double best_score = std::numeric_limits<double>::max();
  PointT best_point;
  bool found = false;

  for (double x = x_lo; x < x_hi; x += params_.search_interval) {
    for (double y = y_lo; y < y_hi; y += params_.search_interval) {

      PointT search_pt;
      search_pt.x = x;
      search_pt.y = y;
      search_pt.z = plane_z;

      double min_dist_to_obs;
      if (has_obstacles) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        if (tree_->nearestKSearch(search_pt, 1, nn_indices, nn_dists) > 0) {
          min_dist_to_obs = std::sqrt(nn_dists[0]);
        } else {
          // No neighbor returned: treat as clear.
          min_dist_to_obs = 100.0;
        }
      } else {
        // If there are no obstacles, any point on the table is perfectly safe
        min_dist_to_obs = 100.0;
      }

      // Gate on guaranteed clearance, then prefer the candidate nearest the robot.
      if (min_dist_to_obs >= params_.obstacle_tolerance) {
        // Record this spot for the placeable-region debug cloud.
        cloud_placeable_->push_back(search_pt);

        double robot_dist = std::hypot(search_pt.x, search_pt.y);
        if (robot_dist < best_score) {
          best_score = robot_dist;
          best_point = search_pt;
          found = true;
        }
      }
    }
  }

  detection_msg_.detections.clear();
  detection_msg_.header = msg->header;

  if (found) {
    vision_msgs::msg::Detection3D det;
    det.header = msg->header;
    det.id = "placeable_pose";
    det.bbox.center.position.x = best_point.x;
    det.bbox.center.position.y = best_point.y;
    det.bbox.center.position.z = best_point.z + 0.01; // Slightly above surface
    det.bbox.center.orientation.w = 1.0; 
    
    // Bounding Box represents the guaranteed safety clearance
    det.bbox.size.x = params_.obstacle_tolerance * 2;
    det.bbox.size.y = params_.obstacle_tolerance * 2;
    det.bbox.size.z = 0.05;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.pose.pose = det.bbox.center;
    hyp.hypothesis.score = 1.0;
    det.results.push_back(hyp);

    detection_msg_.detections.push_back(det);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped t;
    t.header = msg->header;
    t.child_frame_id = "placeable_pose";
    t.transform.translation.x = det.bbox.center.position.x;
    t.transform.translation.y = det.bbox.center.position.y;
    t.transform.translation.z = det.bbox.center.position.z;
    t.transform.rotation = det.bbox.center.orientation;
    tf_broadcaster_->sendTransform(t);
    
    pub_detections_->publish(detection_msg_);
  } else {
    // If we're looking but can't find a spot, let the user/robot know
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No safe placement position found on table.");
  }

  // Debug Cloud Publishing (also runs on every early-return path above).
  publishDebugCloud();
}

}  // namespace pcl_object_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::PlaceableDetectionComponent)
