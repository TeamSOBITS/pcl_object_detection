#ifndef OBJECT_DETECTION_SHELF_NODE_HPP
#define OBJECT_DETECTION_SHELF_NODE_HPP

#include "base_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>




class ObjectDetectionShelfNode : public BaseNode<sensor_msgs::msg::PointCloud2> {
public:
    ObjectDetectionShelfNode(const rclcpp::NodeOptions& options);

    void processData(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    void activate() override;
    void deactivate() override;

private:
    std::shared_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
    bool use_voxel_;
};

#endif // OBJECT_DETECTION_SHELF_NODE_HPP
