#ifndef PCL_OBJECT_DETECTION_HPP
#define PCL_OBJECT_DETECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sobits_interfaces/srv/mode_ctrl.hpp"
#include <memory>
#include <string>
#include <vector>

#include "point_cloud_processor.hpp"
#include "object_detection_table.hpp"
#include "object_detection_floor.hpp"
#include "object_detection_shelf.hpp"
#include "placeable_detection.hpp"

// using pcl_object_detection::PointCloudProcessor;


class PCLNode {
    public:
        rclcpp::Node::SharedPtr nd_;
        // Configure the QoS profile
        rclcpp::QoS qos_profile_; // depth = 1
        std::string pointcloud_topic_name_;

        rclcpp::Service<sobits_interfaces::srv::ModeCtrl>::SharedPtr mode_service_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_cb_handle_;

        PCLNode(std::shared_ptr<rclcpp::Node> nd);

    private:
        void switchModeCallback(const std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Request>  request,
                                      std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Response> response);

        rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
        // 各機能ノード
        std::shared_ptr<ObjectDetectionFloorNode> object_detection_floor_node_;
        std::shared_ptr<ObjectDetectionShelfNode> object_detection_shelf_node_;
        std::shared_ptr<ObjectDetectionTableNode> object_detection_table_node_;
        std::shared_ptr<PlaceableDetectionNode>   placeable_detection_node_;

        // 現在のモード
        int current_mode_;
};

#endif // PCL_OBJECT_DETECTION_HPP
