#include "pcl_object_detection/line_detection.hpp"
#include <pcl_conversions/pcl_conversions.h>

LineDetectionNode::LineDetectionNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd),pcp_(nd){
    nd_->declare_parameter("scan_topic_name","/hsrb/base_scan");
    // nd_->declare_parameter("base_frame_name", "base_footprint");
    nd_->declare_parameter("passthrough_y_min",-1.0);
    nd_->declare_parameter("passthrough_y_max",1.0);
    nd_->declare_parameter("nead_marker",true);
    nd_->declare_parameter("nead_cloud_line",true);
    nd_->declare_parameter("nead_info",true);
    nd_->declare_parameter("execute_flag",true);

    scan_topic_name_ = nd_->get_parameter("scan_topic_name").as_string();
    target_frame_ = nd_->get_parameter("base_frame_name").as_string();
    execute_flag = nd_->get_parameter("execute_flag").as_bool();
    run_ctrl_server_ = nd_->create_service<std_srvs::srv::SetBool>(
          "/pcl_line_detection/run_ctrl", std::bind(&LineDetectionNode::execute_ctrl_server, this, std::placeholders::_1, std::placeholders::_2));

    pub_line_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_line_detection/line_cloud", 10);
    pub_angle_ = nd_->create_publisher<std_msgs::msg::Float64>("/pcl_line_detection/line_angle", 10);
    pub_distance_ = nd_->create_publisher<std_msgs::msg::Float64>("/pcl_line_detection/line_distance", 10);
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS()); // センサーデータ用のQoS
    sub_points_ = nd_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name_,
        sensor_qos,
        std::bind(&LineDetectionNode::processData, this, std::placeholders::_1));
    RCLCPP_INFO(nd_->get_logger(), "LineDetectionNode successfully initialized and ready.");
    RCLCPP_INFO(nd_->get_logger(), "Subscribing to '%s'", this->scan_topic_name_.c_str());
}

void LineDetectionNode::processData(const sensor_msgs::msg::LaserScan::SharedPtr scan2d_msg) {
    if (!execute_flag){return;}
    RCLCPP_INFO(nd_->get_logger(), "processData");
    PointCloud::Ptr cloud_scan2d (new PointCloud());
    PointCloud::Ptr cloud_line( new PointCloud() );
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    auto line_angle_deg = std::make_shared<std_msgs::msg::Float64>();
    // auto info = std::make_shared<pcl_object_detection::msg::LineInfo>();
    Eigen::Vector4f centroid;
    auto angle_deg_ = std_msgs::msg::Float64();
    auto distance_ = std_msgs::msg::Float64();
    pcp_.setPassThroughParameters("y", nd_->get_parameter("passthrough_y_min").as_double(), nd_->get_parameter("passthrough_y_max").as_double());
    pcp_.setSACSegmentationParameter(pcl::SACMODEL_LINE, pcl::SAC_RANSAC);

    if ( !pcp_.transformFrameScan2D2PointCloud( scan2d_msg, cloud_scan2d ) ) return;
    pcp_.passThrough( cloud_scan2d, cloud_scan2d );

    pcp_.sacSegmentation( cloud_scan2d, inliers, coefficients );
    pcp_.extractIndices( cloud_scan2d, cloud_line, inliers, false );
    pcl::compute3DCentroid( *cloud_line, centroid );

    cloud_line->header.frame_id = cloud_scan2d->header.frame_id;
    angle_deg_.data = coefficients->values[3]*(180/M_PI);
    distance_.data = std::hypotf( centroid.x(), centroid.y() );

    RCLCPP_INFO(nd_->get_logger(), "[LineDetection] Angle[deg] = %.2lf, Distance[m] = %.2lf", angle_deg_.data, distance_.data);

    pcl_conversions::toPCL(nd_->get_clock()->now(), cloud_line->header.stamp);

    if ( nd_->get_parameter("nead_cloud_line").as_bool() ) {
        sensor_msgs::msg::PointCloud2 cloud_line_msg;
        pcl::toROSMsg(*cloud_line, cloud_line_msg);
        cloud_line_msg.header.stamp = nd_->now();
        cloud_line_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_line_cloud_->publish(cloud_line_msg);
    }
    if ( nd_->get_parameter("nead_info").as_bool() ) {
        pub_angle_->publish(angle_deg_);
        pub_distance_->publish(distance_);
    }
}


bool LineDetectionNode::execute_ctrl_server(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    execute_flag = req->data;  // Access the boolean request data
    if (execute_flag) {
        RCLCPP_INFO(nd_->get_logger(), "Start Line_Detect.");
        res->message = "Start Line_Detect.";  // Set a message in the response
    } else {
        RCLCPP_INFO(nd_->get_logger(), "Stop Line_Detect.");
        res->message = "Stop Line_Detect.";  // Set a message in the response
    }
    res->success = true;  // Indicate the service call was successful
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pcl_line_detection");
    node->declare_parameter("base_frame_name", "base_footprint");
    node->declare_parameter("publish_cloud_detection_range", true);
    node->declare_parameter("publish_cloud_object", true);
    node->declare_parameter("publish_pose_array", true);
    node->declare_parameter("use_tf", true);

    node->declare_parameter("use_voxel", false);
    node->declare_parameter("leaf_size", 0.01);

    node->declare_parameter("cluster_tolerance", 0.03);
    node->declare_parameter("min_cluster_point_size", 200);
    node->declare_parameter("max_cluster_point_size", 10000);

    node->declare_parameter("threshold_distance", 0.03);
    node->declare_parameter("probability", 0.95);

    node->declare_parameter("object_size_x_min",  0.00);
    node->declare_parameter("object_size_x_max",  0.40);
    node->declare_parameter("object_size_y_min", -0.20);
    node->declare_parameter("object_size_y_max",  0.20);
    node->declare_parameter("object_size_z_min", -0.20);
    node->declare_parameter("object_size_z_max",  0.40);
    // std::make_shared<LineDetectionNode>(node);
    auto line_detection_instance = std::make_shared<LineDetectionNode>(node); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}