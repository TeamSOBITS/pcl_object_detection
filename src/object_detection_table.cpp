#include "pcl_object_detection/object_detection_table.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

ObjectDetectionTableNode::ObjectDetectionTableNode(const rclcpp::NodeOptions& options) : BaseNode<sensor_msgs::msg::PointCloud2>("object_detection_table", options){
    declareCommonParameters();
}

void ObjectDetectionTableNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    PointCloud::Ptr cloud (new PointCloud());
    PointCloud::Ptr cloud_object (new PointCloud());
    auto pose_array = std::make_shared<sobits_interfaces::msg::ObjectPoseArray>();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;
    int object_num = -1;

    pcp_->transformFramePointCloud( cloud_msg, cloud );
    pcp_->passThroughXYZ( cloud );
    if ( use_voxel_ ) pcp_->voxelGrid( cloud, cloud );

    pcp_->setSACPlaneParameter( "z",  5.0 );
    pcp_->sacSegmentation( cloud, inliers, coefficients );
    pcp_->extractIndices( cloud, cloud, inliers, true );

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid( *cloud, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    pcp_->passThrough( cloud, cloud );

    pcl::compute3DCentroid( *cloud, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    pcp_->passThrough( cloud, cloud );

    pcp_->radiusOutlierRemoval( cloud, cloud );
    pcp_->euclideanClusterExtraction ( cloud, &cluster_indices );
    object_num = pcp_->principalComponentAnalysis( cloud, cluster_indices, pose_array, cloud_object );

    pcl_conversions::toPCL(this->get_clock()->now(), cloud->header.stamp);
    pcl_conversions::toPCL(this->get_clock()->now(), cloud_object->header.stamp);

    if ( need_cloud_detection_range_ ) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_cloud_detection_range_->publish(cloud_msg);
    }
    if ( need_cloud_object_ ) {
        sensor_msgs::msg::PointCloud2 cloud_obj_msg;
        pcl::toROSMsg(*cloud_object, cloud_obj_msg);
        cloud_obj_msg.header.stamp = this->now();
        cloud_obj_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_cloud_detection_range_->publish(cloud_obj_msg);
    }
    if ( need_pose_array_ ) pub_pose_array_->publish(*pose_array);

    RCLCPP_INFO(this->get_logger(), "[ObjectDetectionTable] Object count = %d", object_num);
}


void ObjectDetectionTableNode::activate() {

    YAML::Node config = YAML::LoadFile(tabel_param_path_);

    pcp_.reset();
    pcp_ = std::make_shared<pcl_object_detection::PointCloudProcessor>("point_cloud_processor_4");

    use_voxel_ = config["use_voxel"].as<bool>(true);
    use_sobit_pro_ = config["use_sobit_pro"].as<bool>(true);
    
    pcp_->setTargetFrame(config["base_frame_name"].as<std::string>("map"));
    pcp_->setFlag(config["use_tf"].as<bool>(false));
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
    RCLCPP_INFO(this->get_logger(), "ObjectDetectionTableNode activated");
}

void ObjectDetectionTableNode::deactivate() {
    this->sub_.reset();

    this->pub_cloud_detection_range_.reset();
    this->pub_cloud_object_.reset();
    this->pub_pose_array_.reset();
    this->pub_marker_.reset();
    this->pub_line_info_.reset();
    this->pub_cloud_line_.reset();
    // RCLCPP_INFO(this->get_logger(), "ObjectDetectionTableNode deactivated");
}
