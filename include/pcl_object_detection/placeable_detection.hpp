#ifndef PLACEABLE_DETECTION_NODE_HPP
#define PLACEABLE_DETECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include "point_cloud_processor.hpp"

class PlaceableDetectionNode {
    public:
        rclcpp::Node::SharedPtr nd_;
        PlaceableDetectionNode(std::shared_ptr<rclcpp::Node> nd);

        void processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
        void setx_min(double x_min);
        void setx_max(double x_max);
        void sety_min(double y_min);
        void sety_max(double y_max);
        void setz_min(double z_min);
        void setz_max(double z_max);
        void set_obstacle_tolerance(double obstacle_tolerance);
        void set_placeable_search_interval(double placeable_search_interval);

    private:
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_obj_poses_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_placeable_cloud_;

        pcl_object_detection::PointCloudProcessor pcp_;

        double x_min_;
        double x_max_;
        double y_min_;
        double y_max_;
        double z_min_;
        double z_max_;

        double placeable_search_interval_;
        double obstacle_tolerance_;
};

#endif // PLACEABLE_DETECTION_NODE_HPP
