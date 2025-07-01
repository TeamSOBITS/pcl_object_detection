#include "pcl_object_detection/object_detection_floor.hpp"

ObjectDetectionFloorNode::ObjectDetectionFloorNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), pcp_(nd) {
    pub_obj_poses_ = nd_->create_publisher<vision_msgs::msg::Detection3DArray>("object_poses", 5);
    pub_object_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_object", 1);


    // x_min_ = nd_->get_parameter("floor.passthrough_x_min").as_double();
    // x_max_ = nd_->get_parameter("floor.passthrough_x_max").as_double();
    // y_min_ = nd_->get_parameter("floor.passthrough_y_min").as_double();
    // y_max_ = nd_->get_parameter("floor.passthrough_y_max").as_double();
    // z_min_ = nd_->get_parameter("floor.passthrough_z_min").as_double();
    // z_max_ = nd_->get_parameter("floor.passthrough_z_max").as_double();
}

void ObjectDetectionFloorNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(nd_->get_logger(), "ObjectDetectionFloorNode");

    PointCloud::Ptr cloud            (new PointCloud());
    PointCloud::Ptr cloud_object      (new PointCloud());
    auto pose_array = std::make_shared<vision_msgs::msg::Detection3DArray>();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;
    int object_num = -1;


    if (!pcp_.transformFramePointCloud( cloud_msg, cloud )) return;
    pcp_.passThroughXYZ(cloud, 
                        nd_->get_parameter("table.passthrough_x_min").as_double(),
                        nd_->get_parameter("table.passthrough_x_max").as_double(),
                        nd_->get_parameter("table.passthrough_y_min").as_double(),
                        nd_->get_parameter("table.passthrough_y_max").as_double(),
                        nd_->get_parameter("table.passthrough_z_min").as_double(),
                        nd_->get_parameter("table.passthrough_z_max").as_double());
    pcp_.voxelGrid( cloud, cloud );

    pcp_.setSACPlaneParameter("z",5.0);
    pcp_.sacSegmentation( cloud, inliers, coefficients );
    pcp_.extractIndices( cloud, cloud, inliers, true );

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid( *cloud, centroid );
    pcp_.setPassThroughParameters( "z", centroid.z()+0.01, z_max_ );
    pcp_.passThrough( cloud, cloud );

    pcp_.radiusOutlierRemoval( cloud, cloud );
    pcp_.euclideanClusterExtraction ( cloud, &cluster_indices );
    object_num = pcp_.principalComponentAnalysis( cloud, cluster_indices, pose_array, cloud_object );

    RCLCPP_INFO(nd_->get_logger(), "[ObjectDetectionFloor] Object count = %d", object_num);

    cloud->header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    output_cloud_msg.header.stamp = nd_->now();
    output_cloud_msg.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pcl::toROSMsg(*cloud, output_cloud_msg);
    pub_object_cloud_->publish(output_cloud_msg);
}