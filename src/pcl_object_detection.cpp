#include "pcl_object_detection/pcl_object_detection.hpp"

PCLNode::PCLNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), qos_profile_(1) {

    // global param //
    nd_->declare_parameter("base_frame_name", "camera_link");
    nd_->declare_parameter("publish_cloud_detection_range", true);
    nd_->declare_parameter("publish_cloud_object", true);
    nd_->declare_parameter("publish_pose_array", true);
    nd_->declare_parameter("use_tf", true);

    nd_->declare_parameter("use_voxel", false);
    nd_->declare_parameter("leaf_size", 0.01);

    nd_->declare_parameter("cluster_tolerance", 0.03);
    nd_->declare_parameter("min_cluster_point_size", 200);
    nd_->declare_parameter("max_cluster_point_size", 10000);

    nd_->declare_parameter("threshold_distance", 0.03);
    nd_->declare_parameter("probability", 0.95);

    nd_->declare_parameter("object_size_x_min",  0.00);
    nd_->declare_parameter("object_size_x_max",  0.40);
    nd_->declare_parameter("object_size_y_min", -0.20);
    nd_->declare_parameter("object_size_y_max",  0.20);
    nd_->declare_parameter("object_size_z_min", -0.20);
    nd_->declare_parameter("object_size_z_max",  0.40);
    // global param //

    // local param of mode //
    nd_->declare_parameter("table.passthrough_x_min",  0.0);
    nd_->declare_parameter("table.passthrough_x_max",  0.0);
    nd_->declare_parameter("table.passthrough_y_min",  0.0);
    nd_->declare_parameter("table.passthrough_y_max",  0.0);
    nd_->declare_parameter("table.passthrough_z_min",  0.0);
    nd_->declare_parameter("table.passthrough_z_max",  0.0);

    nd_->declare_parameter("floor.passthrough_x_min",  0.0);
    nd_->declare_parameter("floor.passthrough_x_max",  0.0);
    nd_->declare_parameter("floor.passthrough_y_min",  0.0);
    nd_->declare_parameter("floor.passthrough_y_max",  0.0);
    nd_->declare_parameter("floor.passthrough_z_min",  0.0);
    nd_->declare_parameter("floor.passthrough_z_max",  0.0);

    nd_->declare_parameter("shelf.passthrough_x_min",  0.0);
    nd_->declare_parameter("shelf.passthrough_x_max",  0.0);
    nd_->declare_parameter("shelf.passthrough_y_min",  0.0);
    nd_->declare_parameter("shelf.passthrough_y_max",  0.0);
    nd_->declare_parameter("shelf.passthrough_z_min",  0.0);
    nd_->declare_parameter("shelf.passthrough_z_max",  0.0);

    nd_->declare_parameter("placeable.passthrough_x_min",  0.0);
    nd_->declare_parameter("placeable.passthrough_x_max",  0.0);
    nd_->declare_parameter("placeable.passthrough_y_min",  0.0);
    nd_->declare_parameter("placeable.passthrough_y_max",  0.0);
    nd_->declare_parameter("placeable.passthrough_z_min",  0.0);
    nd_->declare_parameter("placeable.passthrough_z_max",  0.0);
    nd_->declare_parameter("placeable.placeable_search_interval",  0.0);
    nd_->declare_parameter("placeable.obstacle_tolerance",  0.0);
    // local param of mode //

    object_detection_table_node_ = std::make_shared<ObjectDetectionTableNode>(nd_);
    object_detection_floor_node_ = std::make_shared<ObjectDetectionFloorNode>(nd_);
    object_detection_shelf_node_ = std::make_shared<ObjectDetectionShelfNode>(nd_);
    placeable_detection_node_ = std::make_shared<PlaceableDetectionNode>(nd_);


    nd_->declare_parameter("pointcloud_topic_name", "/points");
    nd_->declare_parameter("initial_mode", 0);

    pointcloud_topic_name_ = nd_->get_parameter("pointcloud_topic_name").as_string();

    nd_->declare_parameter("qos_profile", "RELIABLE"); /* BEST_EFFORT */

    // Configure the QoS profile
    if      (nd_->get_parameter("qos_profile").as_string() == "RELIABLE")
        qos_profile_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else if (nd_->get_parameter("qos_profile").as_string() == "BEST_EFFORT")
        qos_profile_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    current_mode_ = 0;

    // Create Service
    mode_service_ = nd_->create_service<sobits_interfaces::srv::ModeCtrl>(
        "mode_ctr",
        std::bind(&PCLNode::switchModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    auto request = std::make_shared<sobits_interfaces::srv::ModeCtrl::Request>();
    auto response = std::make_shared<sobits_interfaces::srv::ModeCtrl::Response>();
    request->mode = nd_->get_parameter("initial_mode").as_int();
    switchModeCallback(request, response);
    RCLCPP_INFO(nd_->get_logger(), "Wait for server...");
    rclcpp::spin(nd_);
}


void PCLNode::switchModeCallback(
    const std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Request> request,
    std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Response> response) {

    if ((current_mode_!=request->mode) && ((0 <= request->mode) && (request->mode <= 4))) {
        if (sub_points_) {
            sub_points_.reset();
            sub_points_ = nullptr;
        }
    }

    switch (request->mode) {
        case 0:  // No detection
            RCLCPP_INFO(nd_->get_logger(), "No active mode");
            response->response = true;
            break;
        case 1:  // ObjectDetectionTableNode
            RCLCPP_INFO(nd_->get_logger(), "Activating ObjectDetectionTable...");
            response->response = true;
            if (!sub_points_) sub_points_ = nd_->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name_, qos_profile_, std::bind(&ObjectDetectionTableNode::processData, object_detection_table_node_, std::placeholders::_1));
            break;
        case 2:  // ObjectDetectionFloorNode
            RCLCPP_INFO(nd_->get_logger(), "Activating ObjectDetectionFloor...");
            response->response = true;
            if (!sub_points_) sub_points_ = nd_->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name_, qos_profile_, std::bind(&ObjectDetectionFloorNode::processData, object_detection_floor_node_, std::placeholders::_1));
            break;
        case 3:  // ObjectDetectionShelfNode
            RCLCPP_INFO(nd_->get_logger(), "Activating ObjectDetectionShelf...");
            response->response = true;
            if (!sub_points_) sub_points_ = nd_->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name_, qos_profile_, std::bind(&ObjectDetectionShelfNode::processData, object_detection_shelf_node_, std::placeholders::_1));
            break;
        case 4:  // PlaceableDetectionNode
            RCLCPP_INFO(nd_->get_logger(), "Activating PlaceableDetection...");
            response->response = true;
            if (!sub_points_) sub_points_ = nd_->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name_, qos_profile_, std::bind(&PlaceableDetectionNode::processData, placeable_detection_node_, std::placeholders::_1));
            break;
        default:
            response->response = false;
            break;
    }

    if (response->response) {
        RCLCPP_INFO(nd_->get_logger(), "Successfully switched to mode: %ld", request->mode);
        current_mode_ = request->mode;
    } else {
        RCLCPP_ERROR(nd_->get_logger(), "Failed to switch mode");
    }
    return;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pcl_object_detection");
    std::make_shared<PCLNode>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}