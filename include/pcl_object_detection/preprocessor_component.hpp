#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pcl_object_detection/point_cloud_utility.hpp"


namespace pcl_object_detection {

class PreProcessorComponent : public rclcpp::Node {
public:
  explicit PreProcessorComponent(const rclcpp::NodeOptions & options);

protected:
  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string base_frame_;
  double voxel_leaf_size_;

  // Clipping Bounds
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
};

}  // namespace pcl_object_detection
