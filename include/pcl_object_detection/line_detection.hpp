#ifndef LINE_DETECTION_NODE_HPP
#define LINE_DETECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include "point_cloud_processor.hpp"

class LineDetectionNode : public BaseNode<sensor_msgs::msg::LaserScan> {
    public:
        rclcpp::Node::SharedPtr nd_;
        LineDetectionNode(std::shared_ptr<rclcpp::Node> nd);
        void LineDetectionNode(const rclcpp::NodeOptions& options);

    private:
        std::shared_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
        // std::unique_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
        // pcl_object_detection::PointCloudProcessor pcp_;
};

#endif // LINE_DETECTION_NODE_HPP
