#include "pcl_object_detection/placeable_detection.hpp"

PlaceableDetectionNode::PlaceableDetectionNode(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), pcp_(nd) {
    pub_obj_poses_ = nd_->create_publisher<vision_msgs::msg::Detection3DArray>("object_poses", 5);
    pub_object_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_object", 1);
    pub_placeable_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_detection_range", 1);

    x_min_ = nd_->get_parameter("placeable.passthrough_x_min").as_double();
    x_max_ = nd_->get_parameter("placeable.passthrough_x_max").as_double();
    y_min_ = nd_->get_parameter("placeable.passthrough_y_min").as_double();
    y_max_ = nd_->get_parameter("placeable.passthrough_y_max").as_double();
    z_min_ = nd_->get_parameter("placeable.passthrough_z_min").as_double();
    z_max_ = nd_->get_parameter("placeable.passthrough_z_max").as_double();

    placeable_search_interval_ = nd_->get_parameter("placeable.placeable_search_interval").as_double();
    obstacle_tolerance_ = nd_->get_parameter("placeable.obstacle_tolerance").as_double();
}

void PlaceableDetectionNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    PointCloud::Ptr cloud            (new PointCloud());
    PointCloud::Ptr cloud_plane      (new PointCloud());
    PointCloud::Ptr cloud_plane_hull (new PointCloud());

    // auto pose_array = std::make_shared<sobits_interfaces::msg::ObjectPoseArray>();
    auto pose_array = std::make_shared<vision_msgs::msg::Detection3DArray>();
    pose_array->header.stamp = nd_->now();
    pose_array->header.frame_id = nd_->get_parameter("base_frame_name").as_string();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;

    if (!pcp_.transformFramePointCloud( cloud_msg, cloud )) return;
    pcp_.passThroughXYZ(cloud, x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    pcp_.voxelGrid( cloud, cloud );

    pcp_.setSACPlaneParameter( "z",  5.0 );
    if (!pcp_.sacSegmentation( cloud, inliers, coefficients )) return;
    pcp_.extractIndices( cloud, cloud_plane, inliers, false );
    pcp_.extractIndices( cloud, cloud, inliers, true );
    pcp_.setVoxelGridParameter(); // 0.01
    pcp_.voxelGrid( cloud_plane, cloud_plane );

    Eigen::Vector4f centroid, min_pt, max_pt;
    pcl::compute3DCentroid( *cloud_plane, centroid );
    pcp_.setPassThroughParameters( "z", centroid.z(), centroid.z()+0.4 );
    pcp_.passThrough( cloud, cloud );
    pcp_.setPassThroughParameters( "x", 0.0, max_pt.x() );
    pcp_.passThrough( cloud, cloud );
    // pcl::getMinMax3D( *cloud_plane, min_pt, max_pt);
    // if ( use_sobit_pro_ ) {
    //     pcp_->setPassThroughParameters( "y", 0.0, max_pt.y() );
    //     pcp_->passThrough( cloud, cloud );
    // } else {
    //     pcp_->setPassThroughParameters( "x", 0.0, max_pt.x() );
    //     pcp_->passThrough( cloud, cloud );
    // }

    // Check the number of objects
    pcp_.euclideanClusterExtraction ( cloud, &cluster_indices );
    // int object_num = cluster_indices.size();

    // Obtain plane edges and add object point cloud
    pcp_.voxelGrid( cloud, cloud );
    pcp_.ConcaveHull( cloud_plane, cloud_plane_hull );

    *cloud = *cloud + *cloud_plane_hull;

    // Determine the estimated range of placement locations
    pcp_.setPassThroughParameters( "x", centroid.x() - 0.35, centroid.x() );
    pcp_.passThrough( cloud_plane, cloud_plane );
    pcp_.setPassThroughParameters( "y", centroid.y() - 0.35, centroid.y() + 0.35 );
    pcp_.passThrough( cloud_plane, cloud_plane );
    // if ( use_sobit_pro_ ) {
    //     pcp_->setPassThroughParameters( "x", centroid.x() - 0.35, centroid.x() + 0.35 );
    //     pcp_->passThrough( cloud_plane, cloud_plane );
    //     pcp_->setPassThroughParameters( "y", centroid.y() - 0.35, centroid.y() );
    //     pcp_->passThrough( cloud_plane, cloud_plane );
    // } else {
    //     pcp_->setPassThroughParameters( "x", centroid.x() - 0.35, centroid.x() );
    //     pcp_->passThrough( cloud_plane, cloud_plane );
    //     pcp_->setPassThroughParameters( "y", centroid.y() - 0.35, centroid.y() + 0.35 );
    //     pcp_->passThrough( cloud_plane, cloud_plane );
    // }
    pcl::getMinMax3D( *cloud_plane, min_pt, max_pt);
    pcl::compute3DCentroid( *cloud_plane, centroid );

    geometry_msgs::msg::Point placeable_point;
    double min_pot = 1.0, potential = 0.0;

    geometry_msgs::msg::Point obs_pt;
    for ( double x = max_pt.x() - 0.05; x > min_pt.x() + 0.05; x -= placeable_search_interval_ ) {
        for ( double y = max_pt.y() - 0.05; y > min_pt.y() + 0.05; y -= placeable_search_interval_ ) {
            geometry_msgs::msg::Point search_pt;
            pcl::PointIndices::Ptr nearest_inliers (new pcl::PointIndices);
            search_pt.x = x;
            search_pt.y = y;
            search_pt.z = centroid.z();
            if ( !pcp_.nearestKSearch ( cloud, nearest_inliers, search_pt )) continue;
            obs_pt.x = cloud->points[ nearest_inliers->indices[0] ].x;
            obs_pt.y = cloud->points[ nearest_inliers->indices[0] ].y;
            double obs_dist = std::hypotf( search_pt.x - obs_pt.x, search_pt.y - obs_pt.y );
            if ( obs_dist < obstacle_tolerance_ ) potential = 1.0;
            else potential = ( 1 / ( 1 + obs_dist ));

            if ( min_pot > potential ) {
                min_pot = potential;
                placeable_point = search_pt;
            }
        }
    }

    if ( min_pot != 1.0 ) {
        placeable_point.z += 0.01;
        vision_msgs::msg::Detection3D pose;
        vision_msgs::msg::ObjectHypothesisWithPose ohwp;
        ohwp.hypothesis.class_id = "placeable_point";
        ohwp.hypothesis.score = 1.0;
        ohwp.pose.pose.position.x = placeable_point.x;
        ohwp.pose.pose.position.y = placeable_point.y;
        ohwp.pose.pose.position.z = placeable_point.z;
        ohwp.pose.pose.orientation.x = 0.;
        ohwp.pose.pose.orientation.y = 0.;
        ohwp.pose.pose.orientation.z = 0.;
        ohwp.pose.pose.orientation.w = 1.;
        pose.header.stamp = nd_->now();
        pose.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
        pose.results.push_back(ohwp);
        pose.bbox.center.position.x = placeable_point.x;
        pose.bbox.center.position.y = placeable_point.y;
        pose.bbox.center.position.z = placeable_point.z;
        pose.bbox.center.orientation.x = 0.;
        pose.bbox.center.orientation.y = 0.;
        pose.bbox.center.orientation.z = 0.;
        pose.bbox.center.orientation.w = 1.;
        pose.bbox.size.x = 2 * placeable_search_interval_;
        pose.bbox.size.y = 2 * placeable_search_interval_;
        pose.bbox.size.z = 2 * placeable_search_interval_;
        pose.id = "placeable_point";
        pose_array->detections.push_back(pose);
        pcp_.sendTransform(pose.bbox.center, "placeable_point");
    } else {
        RCLCPP_ERROR(nd_->get_logger(), "NO Placeable Point");
    }

    cloud_plane->header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    cloud->header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pcl_conversions::toPCL(nd_->now(), cloud_plane->header.stamp);
    pcl_conversions::toPCL(nd_->now(), cloud->header.stamp);
    
    sensor_msgs::msg::PointCloud2 cloud_plane_msg;
    pcl::toROSMsg(*cloud_plane, cloud_plane_msg);
    cloud_plane_msg.header.stamp = nd_->now();
    cloud_plane_msg.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pub_object_cloud_->publish(cloud_plane_msg);

    sensor_msgs::msg::PointCloud2 cloud_obj_msg;
    pcl::toROSMsg(*cloud, cloud_obj_msg);
    cloud_obj_msg.header.stamp = nd_->now();
    cloud_obj_msg.header.frame_id = nd_->get_parameter("base_frame_name").as_string();
    pub_placeable_cloud_->publish(cloud_obj_msg);

    pub_obj_poses_->publish(*pose_array);

    RCLCPP_INFO(nd_->get_logger(), "[PlaceablePoseDetection] Object count = %ld", cluster_indices.size());
}

void PlaceableDetectionNode::setx_min(double x_min) {
    x_min_ = x_min;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] x_min updated to: %f", x_min);
}
void PlaceableDetectionNode::setx_max(double x_max) {
    x_max_ = x_max;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] x_max updated to: %f", x_max);
}
void PlaceableDetectionNode::sety_min(double y_min) {
    y_min_ = y_min;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] y_min updated to: %f", y_min);
}
void PlaceableDetectionNode::sety_max(double y_max) {
    y_max_ = y_max;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] y_max updated to: %f", y_max);
}
void PlaceableDetectionNode::setz_min(double z_min) {
    z_min_ = z_min;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] z_min updated to: %f", z_min);
}
void PlaceableDetectionNode::setz_max(double z_max) {
    z_max_ = z_max;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] z_max updated to: %f", z_max);
}
void PlaceableDetectionNode::set_obstacle_tolerance(double obstacle_tolerance) {
    obstacle_tolerance_ = obstacle_tolerance;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] obstacle_tolerance updated to: %f", obstacle_tolerance);
}
void PlaceableDetectionNode::set_placeable_search_interval(double placeable_search_interval) {
    placeable_search_interval_ = placeable_search_interval;
    RCLCPP_INFO(nd_->get_logger(), "[PlaceableDetectionNode] placeable_search_interval updated to: %f", placeable_search_interval);
}