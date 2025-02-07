#ifndef PLACEABLE_DETECTION_NODE_HPP
#define PLACEABLE_DETECTION_NODE_HPP

#include "base_node.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <yaml-cpp/yaml.h>



class PlaceableDetectionNode : public BaseNode<sensor_msgs::msg::PointCloud2> {
public:
    PlaceableDetectionNode(const rclcpp::NodeOptions& options);

    void processData(const sensor_msgs::msg::PointCloud2::SharedPtr msg) override;
    void activate() override;
    void deactivate() override;

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    double placeable_search_interval_;
    double obstacle_tolerance_;
    bool use_voxel_;
    bool use_sobit_pro_;
    std::shared_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
    
};

#endif // PLACEABLE_DETECTION_NODE_HPP
