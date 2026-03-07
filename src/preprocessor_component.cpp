#include "pcl_object_detection/preprocessor_component.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_object_detection {

PreProcessorComponent::PreProcessorComponent(const rclcpp::NodeOptions & options)
: Node("preprocessor", options) {
  
  // Declare Parameters
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.02);

  // Clipping Bounds (Defaults to a generous 10m box)
  x_min_ = this->declare_parameter<double>("x_min", -10.0);
  x_max_ = this->declare_parameter<double>("x_max", 10.0);
  y_min_ = this->declare_parameter<double>("y_min", -10.0);
  y_max_ = this->declare_parameter<double>("y_max", 10.0);
  z_min_ = this->declare_parameter<double>("z_min", -2.0);
  z_max_ = this->declare_parameter<double>("z_max", 5.0);

  RCLCPP_INFO(this->get_logger(), "Parameters Declared");
  RCLCPP_INFO(this->get_logger(), "Base Frame: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Voxel Leaf Size: %f", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "Clipping Bounds:");
  RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", x_min_, x_max_);
  RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", y_min_, y_max_);
  RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", z_min_, z_max_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();
  std::string output_topic = this->declare_parameter<std::string>("output_topic", "cloud_filtered");
  pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic, qos);
  std::string input_topic = this->declare_parameter<std::string>("input_topic", "/camera/depth/color/points");
  sub_raw_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    std::bind(&PreProcessorComponent::cloudCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(this->get_logger(), "PreProcessor Component Initialized");
}

void PreProcessorComponent::cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  // Convert to PCL
  auto cloud_raw = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_raw);

  // Transform to Base Frame
  auto cloud_transformed = std::make_shared<PointCloud>();
  try {
    auto transform = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(base_frame_, *cloud_raw, *cloud_transformed, *tf_buffer_);
    cloud_transformed->header.frame_id = base_frame_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Wait: %s", ex.what());
    return;
  }

  // Safety Clipping (Prevents VoxelGrid overflow)
  auto cloud_clipped = std::make_shared<PointCloud>();
  PointCloudUtility::applyPassThrough(cloud_transformed, cloud_clipped, "x", x_min_, x_max_);
  PointCloudUtility::applyPassThrough(cloud_clipped, cloud_clipped, "y", y_min_, y_max_);
  PointCloudUtility::applyPassThrough(cloud_clipped, cloud_clipped, "z", z_min_, z_max_);

  if (cloud_clipped->empty()) return;

  // Apply Voxel Grid
  auto cloud_downsampled = std::make_shared<PointCloud>();
  PointCloudUtility::applyVoxelGrid(cloud_transformed, cloud_downsampled, voxel_leaf_size_);

  auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud_downsampled, *output_msg);
  output_msg->header.stamp = msg->header.stamp;
  output_msg->header.frame_id = base_frame_;

  pub_filtered_cloud_->publish(std::move(output_msg));
}

}  // namespace pcl_object_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_object_detection::PreProcessorComponent)
