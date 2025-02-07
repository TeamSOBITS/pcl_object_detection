#ifndef OBJECT_DETECTION_FLOOR_NODE_HPP
#define OBJECT_DETECTION_FLOOR_NODE_HPP

#include "base_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <yaml-cpp/yaml.h>



class ObjectDetectionFloorNode : public BaseNode<sensor_msgs::msg::PointCloud2> {
public:
    ObjectDetectionFloorNode(const rclcpp::NodeOptions& options);
    
    void processData(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    void activate() override;
    void deactivate() override;

private:
    bool use_voxel_;
    std::shared_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
};

#endif // OBJECT_DETECTION_FLOOR_NODE_HPP
