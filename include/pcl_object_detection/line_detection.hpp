#ifndef LINE_DETECTION_NODE_HPP
#define LINE_DETECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include "point_cloud_processor.hpp"
#include <std_srvs/srv/set_bool.hpp>


class LineDetectionNode {
    public:
        rclcpp::Node::SharedPtr nd_;
        std::string scan_topic_name_;
        std::string target_frame_;
        bool execute_flag;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_points_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_line_cloud_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_distance_;
        LineDetectionNode(std::shared_ptr<rclcpp::Node> nd);
        void processData(const sensor_msgs::msg::LaserScan::SharedPtr scan_2d_msg);
        bool execute_ctrl_server(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) ;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctrl_server_;
    private:
        pcl_object_detection::PointCloudProcessor pcp_;
        // std::unique_ptr<pcl_object_detection::PointCloudProcessor> pcp_;
};

#endif // LINE_DETECTION_NODE_HPP
