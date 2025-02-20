#include "pcl_object_detection/line_detection.hpp"
#include <pcl_conversions/pcl_conversions.h>

LineDetectionNode::LineDetectionNode(const rclcpp::NodeOptions& options) : BaseNode<sensor_msgs::msg::LaserScan>("line_detection", options) {
    declareCommonParameters();

}

void LineDetectionNode::processData(const sensor_msgs::msg::LaserScan::SharedPtr scan2d_msg) {
    PointCloud::Ptr cloud_scan2d (new PointCloud());
    PointCloud::Ptr cloud_line( new PointCloud() );
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    auto line_angle_deg = std::make_shared<std_msgs::msg::Float64>();
    // auto info = std::make_shared<pcl_object_detection::msg::LineInfo>();
    Eigen::Vector4f centroid;

    if ( !pcp_->transformFrameScan2D2PointCloud( scan2d_msg, cloud_scan2d ) ) return;
    pcp_->passThrough( cloud_scan2d, cloud_scan2d );

    pcp_->sacSegmentation( cloud_scan2d, inliers, coefficients );
    pcp_->extractIndices( cloud_scan2d, cloud_line, inliers, false );
    pcl::compute3DCentroid( *cloud_line, centroid );

    cloud_line->header.frame_id = cloud_scan2d->header.frame_id;
    angle_deg_.data = coefficients->values[3]*(180/M_PI);
    distance_.data = std::hypotf( centroid.x(), centroid.y() );

    RCLCPP_INFO(this->get_logger(), "[LineDetection] Angle[deg] = %.2lf, Distance[m] = %.2lf", angle_deg_.data, distance_.data);


    // info->line_angle_deg = angle_deg_;
    // info->line_distance = distance_;

    pcl_conversions::toPCL(this->get_clock()->now(), cloud_line->header.stamp);
    pcl_conversions::toPCL(this->get_clock()->now(), cloud_line->header.stamp);

    if ( need_cloud_line_ ) {
        sensor_msgs::msg::PointCloud2 cloud_line_msg;
        pcl::toROSMsg(*cloud_line, cloud_line_msg);
        cloud_line_msg.header.stamp = this->now();
        cloud_line_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_cloud_line_->publish(cloud_line_msg);
    }
    // if ( need_line_info_ ) pub_line_info_->publish(*info);
    
    if ( need_marker_ ) {
        std::string str = "Angle[deg] = " + std::to_string(angle_deg_.data) + "  Distance[m] = " + std::to_string(distance_.data);
        pub_marker_->publish(makeMarkerString(str, coefficients->values[0], coefficients->values[1], coefficients->values[2]));
    }
}


visualization_msgs::msg::Marker LineDetectionNode::makeMarkerString(const std::string &string, double x, double y, double z) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = this->now();
    marker.ns = "line_info";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.text = string;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

void LineDetectionNode::activate() {
    YAML::Node config = YAML::LoadFile(line_param_path_);

    need_marker_ = config["need_marker"].as<bool>(true);
    need_cloud_line_ = config["need_cloud_line"].as<bool>(true);
    need_line_info_ = config["need_line_info"].as<bool>(true);


    pcp_.reset();
    pcp_ = std::make_shared<pcl_object_detection::PointCloudProcessor>("point_cloud_processor_1");

    pcp_->setTargetFrame(target_frame_);
    pcp_->setPassThroughParameters("y", config["passthrough_y_min"].as<double>(), config["passthrough_y_max"].as<double>());
    pcp_->setSACSegmentationParameter(pcl::SACMODEL_LINE, pcl::SAC_RANSAC, config["threshold_distance"].as<double>(), config["probability"].as<double>());

    this->setupCommonSubscribers();
    this->setupCommonPublishers();
    RCLCPP_INFO(this->get_logger(), "LineDetectionNode activated");
}

void LineDetectionNode::deactivate() {
    if (this->sub_) {
        this->sub_.reset();
        this->sub_ = nullptr;
    }
    if (this->pcp_) {
        this->pcp_.reset();
        this->pcp_ = nullptr;
    }

    if (this->pub_cloud_detection_range_) {
        this->pub_cloud_detection_range_.reset();
        this->pub_cloud_detection_range_ = nullptr;
    }
    if (this->pub_cloud_object_) {
        this->pub_cloud_object_.reset();
        this->pub_cloud_object_ = nullptr;
    }
    if (this->pub_pose_array_) {
        this->pub_pose_array_.reset();
        this->pub_pose_array_ = nullptr;
    }
    if (this->pub_marker_) {
        this->pub_marker_.reset();
        this->pub_marker_ = nullptr;
    }
    if (this->pub_line_info_) {
        this->pub_line_info_.reset();
        this->pub_line_info_ = nullptr;
    }
    if (this->pub_cloud_line_) {
        this->pub_cloud_line_.reset();
        this->pub_cloud_line_ = nullptr;
    }

    // RCLCPP_INFO(this->get_logger(), "LineDetectionNode deactivated")
}


