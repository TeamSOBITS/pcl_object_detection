#include "pcl_object_detection/object_detection_floor.hpp"

ObjectDetectionFloorNode::ObjectDetectionFloorNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), pcp_(nd) {
    pub_obj_poses_ = nd_->create_publisher<vision_msgs::msg::Detection3DArray>("object_poses", 5);
    pub_object_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_object", 1);


    x_min_ = nd_->get_parameter("floor.passthrough_x_min").as_double();
    x_max_ = nd_->get_parameter("floor.passthrough_x_max").as_double();
    y_min_ = nd_->get_parameter("floor.passthrough_y_min").as_double();
    y_max_ = nd_->get_parameter("floor.passthrough_y_max").as_double();
    z_min_ = nd_->get_parameter("floor.passthrough_z_min").as_double();
    z_max_ = nd_->get_parameter("floor.passthrough_z_max").as_double();
}

void ObjectDetectionFloorNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(nd_->get_logger(), "ObjectDetectionFloorNode");

    PointCloud::Ptr cloud            (new PointCloud());
    PointCloud::Ptr cloud_plane      (new PointCloud());
    PointCloud::Ptr cloud_plane_hull (new PointCloud());

    // auto pose_array = std::make_shared<sobits_interfaces::msg::ObjectPoseArray>();
    auto pose_array = std::make_shared<vision_msgs::msg::Detection3DArray>();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;

    if (!pcp_.transformFramePointCloud( cloud_msg, cloud )) return;
    pcp_.passThroughXYZ(cloud, x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    pcp_.voxelGrid( cloud, cloud );

    // PointCloud::Ptr cloud (new PointCloud());
    // PointCloud::Ptr cloud_object (new PointCloud());
    // auto pose_array = std::make_shared<sobits_interfaces::msg::ObjectPoseArray>();
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // std::vector<pcl::PointIndices> cluster_indices;
    // int object_num = -1;

    // pcp_->transformFramePointCloud( cloud_msg, cloud );
    // pcp_->passThroughXYZ( cloud );
    // if ( use_voxel_ ) pcp_->voxelGrid( cloud, cloud );

    // pcp_->setSACPlaneParameter( "z",  5.0 );
    // pcp_->sacSegmentation( cloud, inliers, coefficients );
    // pcp_->extractIndices( cloud, cloud, inliers, true );

    // Eigen::Vector4f centroid;
    // pcl::compute3DCentroid( *cloud, centroid );
    // pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    // pcp_->passThrough( cloud, cloud );

    // pcl::compute3DCentroid( *cloud, centroid );
    // pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    // pcp_->passThrough( cloud, cloud );

    // pcp_->radiusOutlierRemoval( cloud, cloud );
    // pcp_->euclideanClusterExtraction ( cloud, &cluster_indices );
    // object_num = pcp_->principalComponentAnalysis( cloud, cluster_indices, pose_array, cloud_object );

    // pcl_conversions::toPCL(this->get_clock()->now(), cloud->header.stamp);
    // pcl_conversions::toPCL(this->get_clock()->now(), cloud_object->header.stamp);

    // if ( need_cloud_detection_range_ ) {
    //     sensor_msgs::msg::PointCloud2 cloud_msg;
    //     pcl::toROSMsg(*cloud, cloud_msg);
    //     cloud_msg.header.stamp = this->now();
    //     cloud_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
    //     pub_cloud_detection_range_->publish(cloud_msg);
    // }
    // if ( need_cloud_object_ ) {
    //     sensor_msgs::msg::PointCloud2 cloud_obj_msg;
    //     pcl::toROSMsg(*cloud_object, cloud_obj_msg);
    //     cloud_obj_msg.header.stamp = this->now();
    //     cloud_obj_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
    //     pub_cloud_detection_range_->publish(cloud_obj_msg);
    // }
    // if ( need_pose_array_ ) pub_pose_array_->publish(*pose_array);

    // RCLCPP_INFO(this->get_logger(), "[ObjectDetectionFloor] Object count = %d", object_num);

    cloud->header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    output_cloud_msg.header.stamp = nd_->now();
    output_cloud_msg.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pcl::toROSMsg(*cloud, output_cloud_msg);
    pub_object_cloud_->publish(output_cloud_msg);
}