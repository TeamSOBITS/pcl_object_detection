#ifndef LINE_DETECTION_NODE_HPP
#define LINE_DETECTION_NODE_HPP

#include "base_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <yaml-cpp/yaml.h>



class LineDetectionNode : public BaseNode<sensor_msgs::msg::LaserScan> {
public:
    explicit LineDetectionNode(const rclcpp::NodeOptions& options);

    void processData(const sensor_msgs::msg::LaserScan::SharedPtr msg) override;
    void activate() override;
    void deactivate() override;

private:

    bool need_marker_;
    bool need_cloud_line_;
    bool need_line_info_;

    visualization_msgs::msg::Marker makeMarkerString(const std::string &string, double x, double y, double z);
    std::shared_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
    std_msgs::msg::Float64 angle_deg_;
    std_msgs::msg::Float64 distance_;
};

#endif // LINE_DETECTION_NODE_HPP
