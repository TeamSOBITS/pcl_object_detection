#include "pcl_object_detection/mode_manager_node.hpp"

ModeManagerNode::ModeManagerNode()
    : rclcpp::Node("mode_manager_node"), current_mode_(0) {

    auto options = rclcpp::NodeOptions();

    // 各機能ノードを初期化
    line_detection_node_ = std::shared_ptr<LineDetectionNode>(new LineDetectionNode(options));
    object_detection_floor_node_ = std::shared_ptr<ObjectDetectionFloorNode>(new ObjectDetectionFloorNode(options));
    object_detection_shelf_node_ = std::shared_ptr<ObjectDetectionShelfNode>(new ObjectDetectionShelfNode(options));
    object_detection_table_node_ = std::shared_ptr<ObjectDetectionTableNode>(new ObjectDetectionTableNode(options));
    placeable_detection_node_ = std::shared_ptr<PlaceableDetectionNode>(new PlaceableDetectionNode(options));


    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(line_detection_node_);
    executor_->add_node(object_detection_floor_node_);
    executor_->add_node(object_detection_shelf_node_);
    executor_->add_node(object_detection_table_node_);
    executor_->add_node(placeable_detection_node_);

    // スレッドを作成して executor を管理
    executor_thread_ = std::thread([this]() {
        executor_->spin();
    });
    
    // サービスを作成
    mode_service_ = this->create_service<sobits_interfaces::srv::ModeCtrl>(
        "switch_mode",
        std::bind(&ModeManagerNode::switchModeCallback, this, std::placeholders::_1, std::placeholders::_2));
}


ModeManagerNode::~ModeManagerNode() {
    executor_->cancel();
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
}

void ModeManagerNode::deactivateAllNodes() {
    RCLCPP_INFO(this->get_logger(), "Deactivating all nodes...");
    std::cout << "===============================================" << std::endl;
    
    line_detection_node_->deactivate();
    object_detection_floor_node_->deactivate();
    object_detection_shelf_node_->deactivate();
    object_detection_table_node_->deactivate();
    placeable_detection_node_->deactivate();
}

void ModeManagerNode::switchModeCallback(
    const std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Request> request,
    std::shared_ptr<sobits_interfaces::srv::ModeCtrl::Response> response){
    int64_t mode = request->mode;

    if (mode < 0 || mode > 5) {
        response->response = false;
        RCLCPP_WARN(this->get_logger(), "Invalid mode requested: %ld", mode);
        return;
    }

    // deactivateAllNodes();
    current_mode_ = mode;

    executor_->cancel();  // Executorを停止
    deactivateAllNodes();  // すべてのノードを無効化
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    switch (mode) {
        case 0:  // 使用しない
            RCLCPP_INFO(this->get_logger(), "No active mode");
            response->response = true;
            break;
        case 1:  // LineDetectionNode
            RCLCPP_INFO(this->get_logger(), "Activating LineDetectionNode...");
            line_detection_node_->activate();
            response->response = true;
            break;
        case 2:  // ObjectDetectionFloorNode
            RCLCPP_INFO(this->get_logger(), "Activating ObjectDetectionFloorNode...");
            object_detection_floor_node_->activate();
            response->response = true;
            break;
        case 3:  // ObjectDetectionShelfNode
            RCLCPP_INFO(this->get_logger(), "Activating ObjectDetectionShelfNode...");
            object_detection_shelf_node_->activate();
            response->response = true;
            break;
        case 4:  // ObjectDetectionTableNode
            RCLCPP_INFO(this->get_logger(), "Activating ObjectDetectionTableNode...");
            object_detection_table_node_->activate();
            response->response = true;
            break;
        case 5:  // PlaceableDetectionNode
            RCLCPP_INFO(this->get_logger(), "Activating PlaceableDetectionNode...");
            placeable_detection_node_->activate();
            response->response = true;
            break;
        default:
            response->response = false;
            break;
    }

    if (response->response) {
        RCLCPP_INFO(this->get_logger(), "Successfully switched to mode: %ld", mode);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch mode");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModeManagerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}