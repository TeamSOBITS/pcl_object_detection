#include "pcl_object_detection/object_detection_shelf.hpp"
#include <pcl_conversions/pcl_conversions.h>

ObjectDetectionShelfNode::ObjectDetectionShelfNode(const rclcpp::NodeOptions& options) : BaseNode<sensor_msgs::msg::PointCloud2>("object_detection_shelf", options){
    declareCommonParameters();
}

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

void ObjectDetectionShelfNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(this->get_logger(),"\033[1;32m[ ObjectDetectionShelf ] \033[m");
}

void ObjectDetectionShelfNode::activate() {

    YAML::Node config = YAML::LoadFile(shelf_param_path_);

    use_voxel_ = config["use_voxel"].as<bool>(true);

    pcp_.reset();
    pcp_ = std::make_shared<pcl_object_detection::PointCloudProcessor>("point_cloud_processor_3");

    pcp_->setTargetFrame(target_frame_);
    pcp_->setFlag(use_tf_);
    pcp_->setPassThroughParameters(
        config["passthrough_x_min"].as<double>(-1.0), config["passthrough_x_max"].as<double>(1.0),
        config["passthrough_y_min"].as<double>(-1.0), config["passthrough_y_max"].as<double>(1.0),
        config["passthrough_z_min"].as<double>(-1.0), config["passthrough_z_max"].as<double>(1.0));
    pcp_->setVoxelGridParameter(config["leaf_size"].as<double>(0.01));
    pcp_->setClusteringParameters(
        config["cluster_tolerance"].as<double>(0.02),
        config["min_cluster_point_size"].as<int>(10),
        config["max_cluster_point_size"].as<int>(1000));
    pcp_->setSACSegmentationParameter(
        pcl::SACMODEL_PERPENDICULAR_PLANE, pcl::SAC_RANSAC,
        config["threshold_distance"].as<double>(0.01), config["probability"].as<double>(0.99));
    pcp_->setObjectSizeParameter(
        config["object_size_x_min"].as<double>(0.1), config["object_size_x_max"].as<double>(1.0),
        config["object_size_y_min"].as<double>(0.1), config["object_size_y_max"].as<double>(1.0),
        config["object_size_z_min"].as<double>(0.1), config["object_size_z_max"].as<double>(1.0));
    pcp_->setObjectOffsetParameter(
        config["object_centroid_offset_x"].as<double>(0.0),
        config["object_centroid_offset_y"].as<double>(0.0),
        config["object_centroid_offset_z"].as<double>(0.0));

    this->setupCommonSubscribers();
    this->setupCommonPublishers();
    RCLCPP_INFO(this->get_logger(), "ObjectDetectionShelfNode activated");
}

void ObjectDetectionShelfNode::deactivate() {
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
    // RCLCPP_INFO(this->get_logger(), "ObjectDetectionShelfNode deactivated");
}
