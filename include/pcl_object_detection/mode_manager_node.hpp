#ifndef MODE_MANAGER_NODE_HPP
#define MODE_MANAGER_NODE_HPP

#include "base_node.hpp"
#include "line_detection.hpp"
#include "object_detection_shelf.hpp"
#include "object_detection_floor.hpp"
#include "object_detection_table.hpp"
#include "placeable_detection.hpp"
// #include "modectrl/srv/modectrl.hpp"
#include <sobits_interfaces/srv/mode_ctrl.hpp>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <unistd.h>


class ModeManagerNode : public rclcpp::Node {
public:
    ModeManagerNode();
    ~ModeManagerNode();

private:
    void switchModeCallback(
        const std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Request> request,
        std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Response> response);

    void deactivateAllNodes();

    // 各機能ノード
    // rclcpp::Node::SharedPtr line_detection_node_;
    // rclcpp::Node::SharedPtr object_detection_floor_node_;
    // rclcpp::Node::SharedPtr object_detection_shelf_node_;
    // rclcpp::Node::SharedPtr object_detection_table_node_;
    // rclcpp::Node::SharedPtr placeable_detection_node_;

    std::shared_ptr<LineDetectionNode> line_detection_node_;
    std::shared_ptr<ObjectDetectionFloorNode> object_detection_floor_node_;
    std::shared_ptr<ObjectDetectionShelfNode> object_detection_shelf_node_;
    std::shared_ptr<ObjectDetectionTableNode> object_detection_table_node_;
    std::shared_ptr<PlaceableDetectionNode> placeable_detection_node_;

    // 現在のモード
    int64_t current_mode_;

    // サービス
    rclcpp::Service<sobits_interfaces::srv::ModeCtrl>::SharedPtr mode_service_;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
};

#endif // MODE_MANAGER_NODE_HPP
