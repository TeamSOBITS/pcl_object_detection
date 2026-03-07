#include "pcl_object_detection/object_detection_shelf.hpp"

ObjectDetectionShelfNode::ObjectDetectionShelfNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), pcp_(nd) {
    pub_obj_poses_ = nd_->create_publisher<vision_msgs::msg::Detection3DArray>("object_poses", 5);
    pub_object_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_object", 1);


    x_min_ = nd_->get_parameter("shelf.passthrough_x_min").as_double();
    x_max_ = nd_->get_parameter("shelf.passthrough_x_max").as_double();
    y_min_ = nd_->get_parameter("shelf.passthrough_y_min").as_double();
    y_max_ = nd_->get_parameter("shelf.passthrough_y_max").as_double();
    z_min_ = nd_->get_parameter("shelf.passthrough_z_min").as_double();
    z_max_ = nd_->get_parameter("shelf.passthrough_z_max").as_double();
}

void ObjectDetectionShelfNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(nd_->get_logger(), "ObjectDetectionShelfNode");

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

    // RCLCPP_INFO(this->get_logger(),"\033[1;32m[ ObjectDetectionShelf ] \033[m");

    cloud->header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    output_cloud_msg.header.stamp = nd_->now();
    output_cloud_msg.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pcl::toROSMsg(*cloud, output_cloud_msg);
    pub_object_cloud_->publish(output_cloud_msg);
}