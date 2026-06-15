#include "pcl_object_detection/basket_detection_component.hpp"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pcl_object_detection {

BasketDetectionComponent::BasketDetectionComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("basket_detection", options) {
  this->declare_parameter<std::string>("input_topic", "filtered_cloud");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<double>("detection_height_min", 0.05);
  this->declare_parameter<double>("detection_height_max", 0.60);
  this->declare_parameter<double>("cluster_tolerance", 0.05);
  this->declare_parameter<int>("min_cluster_size", 200);
  this->declare_parameter<int>("max_cluster_size", 10000);

  this->declare_parameter<double>("basket_width_min", 0.40);
  this->declare_parameter<double>("basket_width_max", 0.70);
  this->declare_parameter<double>("basket_depth_min", 0.30);
  this->declare_parameter<double>("basket_depth_max", 0.50);
  this->declare_parameter<double>("basket_height_min", 0.15);
  this->declare_parameter<double>("basket_height_max", 0.40);

  this->declare_parameter<double>("handle_search_radius", 0.05);

  this->declare_parameter<bool>("cloth_detection_enabled", true);
  this->declare_parameter<double>("cloth_inner_margin", 0.08);

  this->declare_parameter<std::string>("cloud_reliability", "best_effort");
  this->declare_parameter<std::string>("detections_pub_reliability", "reliable");
  this->declare_parameter<std::string>("debug_pub_reliability", "best_effort");
}

BasketDetectionComponent::CallbackReturn BasketDetectionComponent::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring Basket Detection...");

  try {
    params_.base_frame = this->get_parameter("base_frame").as_string();
    params_.detection_height_min = this->get_parameter("detection_height_min").as_double();
    params_.detection_height_max = this->get_parameter("detection_height_max").as_double();
    params_.cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    params_.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    params_.max_cluster_size = this->get_parameter("max_cluster_size").as_int();

    params_.basket_width_min = this->get_parameter("basket_width_min").as_double();
    params_.basket_width_max = this->get_parameter("basket_width_max").as_double();
    params_.basket_depth_min = this->get_parameter("basket_depth_min").as_double();
    params_.basket_depth_max = this->get_parameter("basket_depth_max").as_double();
    params_.basket_height_min = this->get_parameter("basket_height_min").as_double();
    params_.basket_height_max = this->get_parameter("basket_height_max").as_double();

    params_.handle_search_radius = this->get_parameter("handle_search_radius").as_double();

    params_.cloth_detection_enabled = this->get_parameter("cloth_detection_enabled").as_bool();
    params_.cloth_inner_margin = this->get_parameter("cloth_inner_margin").as_double();

    cloud_reliability_ = this->get_parameter("cloud_reliability").as_string();
    detections_pub_reliability_ = this->get_parameter("detections_pub_reliability").as_string();
    debug_pub_reliability_ = this->get_parameter("debug_pub_reliability").as_string();
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter type mismatch: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error during configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  cloud_filtered_ = std::make_shared<PointCloud>();
  cloud_basket_candidates_ = std::make_shared<PointCloud>();
  debug_cloud_ = std::make_shared<PointCloud>();
  tree_ = std::make_shared<pcl::search::KdTree<PointT>>();

  ec_.setSearchMethod(tree_);

  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };

  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("basket_objects", make_qos(detections_pub_reliability_, 10));
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("basket_debug_cloud", make_qos(debug_pub_reliability_, 10));

  RCLCPP_INFO(this->get_logger(), "detections pub reliability: %s", detections_pub_reliability_.c_str());
  RCLCPP_INFO(this->get_logger(), "debug pub reliability: %s", debug_pub_reliability_.c_str());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

BasketDetectionComponent::CallbackReturn BasketDetectionComponent::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating Basket Detection...");

  pub_detections_->on_activate();
  pub_debug_cloud_->on_activate();

  auto make_qos = [](const std::string & reliability, size_t depth) -> rclcpp::QoS {
    auto q = rclcpp::QoS(rclcpp::KeepLast(depth));
    q.reliability(reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    return q;
  };

  std::string input_topic = this->get_parameter("input_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "cloud subscription reliability: %s", cloud_reliability_.c_str());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_filtered_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, make_qos(cloud_reliability_, 10),
    std::bind(&BasketDetectionComponent::cloudCallback, this, std::placeholders::_1),
    sub_options);

  return CallbackReturn::SUCCESS;
}

BasketDetectionComponent::CallbackReturn BasketDetectionComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating Basket Detection...");
  pub_detections_->on_deactivate();
  pub_debug_cloud_->on_deactivate();
  sub_filtered_cloud_.reset();
  return CallbackReturn::SUCCESS;
}

BasketDetectionComponent::CallbackReturn BasketDetectionComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up Basket Detection...");
  pub_detections_.reset();
  pub_debug_cloud_.reset();
  tf_broadcaster_.reset();
  cloud_filtered_.reset();
  cloud_basket_candidates_.reset();
  tree_.reset();
  return CallbackReturn::SUCCESS;
}

BasketDetectionComponent::CallbackReturn BasketDetectionComponent::on_shutdown(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

void BasketDetectionComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  cloud_filtered_->clear();
  cloud_basket_candidates_->clear();

  pcl::fromROSMsg(*msg, *cloud_filtered_);
  if (cloud_filtered_->empty()) return;

  // Filter by height to isolate basket
  PointCloudUtility::applyPassThrough(
    cloud_filtered_, cloud_basket_candidates_, "z", 
    params_.detection_height_min, params_.detection_height_max);

  if (cloud_basket_candidates_->empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No points in height range [%f, %f]", params_.detection_height_min, params_.detection_height_max);
    return;
  }

  // Euclidean Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  ec_.setClusterTolerance(params_.cluster_tolerance);
  ec_.setMinClusterSize(params_.min_cluster_size);
  ec_.setMaxClusterSize(params_.max_cluster_size);
  ec_.setInputCloud(cloud_basket_candidates_);
  ec_.extract(cluster_indices);

  if (cluster_indices.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No clusters found in %zu points", cloud_basket_candidates_->size());
  }

  detection_msg_.detections.clear();
  detection_msg_.header = msg->header;
  debug_cloud_->clear();

  int basket_count = 0;
  for (const auto & cluster : cluster_indices) {
    auto box = PointCloudUtility::computePCAAlignedBox(cloud_basket_candidates_, cluster);

    double actual_width = box.size.x;
    double actual_depth = box.size.y;
    double actual_height = box.size.z;

    // Validate size
    if (actual_width < params_.basket_width_min || actual_width > params_.basket_width_max ||
        actual_depth < params_.basket_depth_min || actual_depth > params_.basket_depth_max ||
        actual_height < params_.basket_height_min || actual_height > params_.basket_height_max) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Rejected cluster at [%.2f, %.2f, %.2f]: size [%.2f, %.2f, %.2f]", 
        box.center.position.x, box.center.position.y, box.center.position.z,
        actual_width, actual_depth, actual_height);
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Basket found! Size: [%f, %f, %f]", actual_width, actual_depth, actual_height);

    // --- Optimization: Isolate Basket Cluster ---
    PointCloud::Ptr basket_cloud(new PointCloud);
    pcl::copyPointCloud(*cloud_basket_candidates_, cluster.indices, *basket_cloud);

    // Identify handle points
    // Extract rotation to project points
    double yaw = 2.0 * std::atan2(box.center.orientation.z, box.center.orientation.w);
    Eigen::Vector3f center(box.center.position.x, box.center.position.y, box.center.position.z);
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    // First pass: find max Z in left/right areas
    float max_z_left = -1e6, max_z_right = -1e6;
    for (const auto& point : basket_cloud->points) {
        Eigen::Vector3f pt = point.getVector3fMap();
        Eigen::Vector3f local_pt = rotation.transpose() * (pt - center);

        if (local_pt.x() > (actual_width / 2.0 - params_.handle_search_radius)) {
            max_z_left = std::max(max_z_left, pt.z());
        }
        if (local_pt.x() < (-actual_width / 2.0 + params_.handle_search_radius)) {
            max_z_right = std::max(max_z_right, pt.z());
        }
    }

    // Second pass: compute centroid of points near max_z in each area
    Eigen::Vector3f left_sum(0,0,0), right_sum(0,0,0);
    int left_count = 0, right_count = 0;
    float rim_height_threshold = 0.03;

    for (const auto& point : basket_cloud->points) {
        Eigen::Vector3f pt = point.getVector3fMap();
        Eigen::Vector3f local_pt = rotation.transpose() * (pt - center);

        if (local_pt.x() > (actual_width / 2.0 - params_.handle_search_radius) && 
            pt.z() > (max_z_left - rim_height_threshold)) {
            left_sum += pt;
            left_count++;
        }
        if (local_pt.x() < (-actual_width / 2.0 + params_.handle_search_radius) && 
            pt.z() > (max_z_right - rim_height_threshold)) {
            right_sum += pt;
            right_count++;
        }
    }

    Eigen::Vector3f handle_left_local(0,0,0), handle_right_local(0,0,0);
    if (left_count > 0) {
        Eigen::Vector3f centroid_g = left_sum / static_cast<float>(left_count);
        handle_left_local = rotation.transpose() * (centroid_g - center);
        handle_left_local = Eigen::Vector3f(handle_left_local.x(), 0.0f, handle_left_local.z());
    }
    if (right_count > 0) {
        Eigen::Vector3f centroid_g = right_sum / static_cast<float>(right_count);
        handle_right_local = rotation.transpose() * (centroid_g - center);
        handle_right_local = Eigen::Vector3f(handle_right_local.x(), 0.0f, handle_right_local.z());
    }

    // Sort handles relative to robot's base_footprint (Y axis)
    // Ensure handle_left_local is the one that results in a more positive Y global coordinate
    Eigen::Vector3f global_l = center + rotation * handle_left_local;
    Eigen::Vector3f global_r = center + rotation * handle_right_local;
    if (global_l.y() < global_r.y()) {
        std::swap(handle_left_local, handle_right_local);
    }

    // Create Detection Message
    vision_msgs::msg::Detection3D det;
    det.header = msg->header;
    det.bbox = box;
    det.id = "basket_" + std::to_string(basket_count);
    
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.pose.pose = det.bbox.center;
    hyp.hypothesis.score = 1.0;
    det.results.push_back(hyp);
    detection_msg_.detections.push_back(det);

    // Broadcast TFs
    geometry_msgs::msg::TransformStamped t;
    t.header = msg->header;
    
    // Basket Center (Use adjusted orientation to align axes with handles)
    t.child_frame_id = det.id;
    t.transform.translation.x = box.center.position.x;
    t.transform.translation.y = box.center.position.y;
    t.transform.translation.z = box.center.position.z;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

    // Handles (Broadcasted relative to the basket center for perfect alignment)
    t.header.frame_id = det.id;
    
    // Left Handle (+X edge): Same orientation as Right (consistent global axes)
    t.child_frame_id = det.id + "_handle_left";
    t.transform.translation.x = handle_left_local.x();
    t.transform.translation.y = handle_left_local.y();
    t.transform.translation.z = handle_left_local.z();
    
    // Unified Orientation: X down (red), Y backward (green), Z left (blue)
    // This flips Z exactly 180 deg as requested while maintaining right-hand rule
    tf2::Vector3 x_axis(0, 0, -1); // Down
    tf2::Vector3 y_axis(-1, 0, 0); // Backward
    tf2::Vector3 z_axis(0, 1, 0);  // Left (along handle)
    
    tf2::Matrix3x3 m_common(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z()
    );
    
    tf2::Quaternion q_handle;
    m_common.getRotation(q_handle);
    t.transform.rotation = tf2::toMsg(q_handle);
    tf_broadcaster_->sendTransform(t);

    // Right Handle (-X edge): Same orientation
    t.child_frame_id = det.id + "_handle_right";
    t.transform.translation.x = handle_right_local.x();
    t.transform.translation.y = handle_right_local.y();
    t.transform.translation.z = handle_right_local.z();
    
    tf_broadcaster_->sendTransform(t);

    // --- Cloth Detection Phase ---
    if (params_.cloth_detection_enabled) {
      float cloth_max_z = -1e6;
      int cloth_max_idx = -1;
      float inner_x = actual_width / 2.0 - params_.cloth_inner_margin;
      float inner_y = actual_depth / 2.0 - params_.cloth_inner_margin;

      // Extract points inside the basket volume
      pcl::PointIndices::Ptr cloth_indices(new pcl::PointIndices());
      for (size_t i = 0; i < basket_cloud->points.size(); ++i) {
        Eigen::Vector3f pt = basket_cloud->points[i].getVector3fMap();
        Eigen::Vector3f local_pt = rotation.transpose() * (pt - center);

        if (std::abs(local_pt.x()) < inner_x && std::abs(local_pt.y()) < inner_y) {
          cloth_indices->indices.push_back(i);
          if (pt.z() > cloth_max_z) {
            cloth_max_z = pt.z();
            cloth_max_idx = i;
          }
        }
      }

      if (cloth_max_idx != -1) {
        // Compute Normal at the highest point
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(basket_cloud);
        ne.setIndices(cloth_indices);
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.05); // 5cm neighborhood
        ne.compute(*normal_cloud);

        // Find the index in the normal cloud (matching the input cloud indices)
        // Since we provided cloth_indices, we need to map the max_idx back
        int local_idx = -1;
        for(size_t i=0; i<cloth_indices->indices.size(); ++i) {
          if(cloth_indices->indices[i] == static_cast<int>(cloth_max_idx)) {
            local_idx = i;
            break;
          }
        }

        tf2::Vector3 normal(0, 0, 1);
        if (local_idx != -1 && std::isfinite(normal_cloud->points[local_idx].normal_z)) {
          normal.setValue(
            normal_cloud->points[local_idx].normal_x,
            normal_cloud->points[local_idx].normal_y,
            normal_cloud->points[local_idx].normal_z
          );
          // Ensure normal points upward
          if (normal.z() < 0) normal = -normal;
        }

        // Create Pinch TF
        t.child_frame_id = det.id + "_cloth";
        t.transform.translation.x = basket_cloud->points[cloth_max_idx].x;
        t.transform.translation.y = basket_cloud->points[cloth_max_idx].y;
        t.transform.translation.z = basket_cloud->points[cloth_max_idx].z;
        t.header.frame_id = params_.base_frame; // Broadcast relative to base for simplicity

        // Orientation: X-approach points opposite to normal
        tf2::Vector3 approach = -normal;
        tf2::Vector3 lateral;
        
        if (std::abs(approach.dot(tf2::Vector3(0, 0, 1))) > 0.9) {
          // If approach is vertical, use basket yaw to define Y/Z
          lateral = tf2::Vector3(-std::sin(yaw), std::cos(yaw), 0);
        } else {
          // Flatten onto horizontal plane for lateral orientation
          lateral = approach.cross(tf2::Vector3(0, 0, 1)).normalized();
        }
        
        tf2::Vector3 up = lateral.cross(approach).normalized();
        tf2::Matrix3x3 m_pinch(
          approach.x(), up.x(), lateral.x(),
          approach.y(), up.y(), lateral.y(),
          approach.z(), up.z(), lateral.z()
        );
        
        tf2::Quaternion q_pinch;
        m_pinch.getRotation(q_pinch);
        t.transform.rotation = tf2::toMsg(q_pinch);
        tf_broadcaster_->sendTransform(t);
      }
    }

    basket_count++;
    
    // Only process the first valid basket as requested (expected only one)
    // Extract only this cluster for the debug cloud
    pcl::copyPointCloud(*cloud_basket_candidates_, cluster.indices, *debug_cloud_);
    break; 
  }

  pub_detections_->publish(detection_msg_);

  // Debug Cloud Publishing (Only current basket)
  if (pub_debug_cloud_->get_subscription_count() > 0 && !debug_cloud_->empty()) {
    auto debug_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*debug_cloud_, *debug_msg);
    debug_msg->header = msg->header;
    pub_debug_cloud_->publish(std::move(debug_msg));
  }
}

}  // namespace pcl_object_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::BasketDetectionComponent)
