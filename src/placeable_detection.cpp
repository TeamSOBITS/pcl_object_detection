#include "pcl_object_detection/placeable_detection.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

PlaceableDetectionNode::PlaceableDetectionNode(const rclcpp::NodeOptions& options) : BaseNode<sensor_msgs::msg::PointCloud2>("placeable_detection", options){
    declareCommonParameters();
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void PlaceableDetectionNode::processData(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    PointCloud::Ptr cloud (new PointCloud());
    PointCloud::Ptr cloud_plane (new PointCloud());
    PointCloud::Ptr cloud_plane_hull (new PointCloud());
    auto pose_array = std::make_shared<sobits_interfaces::msg::ObjectPoseArray>();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;

    pcp_->transformFramePointCloud( cloud_msg, cloud );
    pcp_->passThroughXYZ( cloud );
    if ( use_voxel_ ) pcp_->voxelGrid( cloud, cloud );

    pcp_->setSACPlaneParameter( "z",  5.0 );
    pcp_->sacSegmentation( cloud, inliers, coefficients );
    pcp_->extractIndices( cloud, cloud_plane, inliers, false );
    pcp_->extractIndices( cloud, cloud, inliers, true );
    pcp_->setVoxelGridParameter( 0.01 );
    pcp_->voxelGrid( cloud_plane, cloud_plane );

    Eigen::Vector4f centroid, min_pt, max_pt;
    pcl::compute3DCentroid( *cloud_plane, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z(), centroid.z()+0.4 );
    pcp_->passThrough( cloud, cloud );
    pcl::getMinMax3D( *cloud_plane, min_pt, max_pt);
    if ( use_sobit_pro_ ) {
        pcp_->setPassThroughParameters( "y", 0.0, max_pt.y() );
        pcp_->passThrough( cloud, cloud );
    } else {
        pcp_->setPassThroughParameters( "x", 0.0, max_pt.x() );
        pcp_->passThrough( cloud, cloud );
    }

    // Check the number of objects
    pcp_->euclideanClusterExtraction ( cloud, &cluster_indices );
    int object_num = cluster_indices.size();

    // Obtain plane edges and add object point cloud
    pcp_->voxelGrid( cloud, cloud );
    pcp_->ConcaveHull( cloud_plane, cloud_plane_hull );

    *cloud = *cloud + *cloud_plane_hull;

    // Determine the estimated range of placement locations
    if ( use_sobit_pro_ ) {
        pcp_->setPassThroughParameters( "x", centroid.x() - 0.35, centroid.x() + 0.35 );
        pcp_->passThrough( cloud_plane, cloud_plane );
        pcp_->setPassThroughParameters( "y", centroid.y() - 0.35, centroid.y() );
        pcp_->passThrough( cloud_plane, cloud_plane );
    } else {
        pcp_->setPassThroughParameters( "x", centroid.x() - 0.35, centroid.x() );
        pcp_->passThrough( cloud_plane, cloud_plane );
        pcp_->setPassThroughParameters( "y", centroid.y() - 0.35, centroid.y() + 0.35 );
        pcp_->passThrough( cloud_plane, cloud_plane );
    }
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
            if ( !pcp_->nearestKSearch ( cloud, nearest_inliers, search_pt )) continue;
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
        sobits_interfaces::msg::ObjectPose pose;
        pose.class_name = "placeable_point";
        pose.pose.position = placeable_point;
        pose_array->object_poses.push_back(pose);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = target_frame_;
        transformStamped.child_frame_id = "placeable_point";
        transformStamped.transform.translation.x = placeable_point.x;
        transformStamped.transform.translation.y = placeable_point.y;
        transformStamped.transform.translation.z = placeable_point.z;

        if (!broadcaster_) {
            RCLCPP_ERROR(this->get_logger(), "broadcaster_ is nullptr!");
            return;
        }
        

        broadcaster_->sendTransform(transformStamped);
    } else {
        RCLCPP_ERROR(this->get_logger(), "NO placeable_point");
    }

    cloud_plane->header.frame_id = target_frame_;
    cloud->header.frame_id = target_frame_;
    pcl_conversions::toPCL(this->get_clock()->now(), cloud_plane->header.stamp);
    pcl_conversions::toPCL(this->get_clock()->now(), cloud->header.stamp);

    if ( need_cloud_detection_range_ ) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_plane, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_cloud_detection_range_->publish(cloud_msg);
    }
    if ( need_cloud_object_ ) {
        sensor_msgs::msg::PointCloud2 cloud_obj_msg;
        pcl::toROSMsg(*cloud, cloud_obj_msg);
        cloud_obj_msg.header.stamp = this->now();
        cloud_obj_msg.header.frame_id = target_frame_;  // 必要に応じてフレームIDを設定
        pub_cloud_detection_range_->publish(cloud_obj_msg);
    }
    if ( need_pose_array_ ) pub_pose_array_->publish(*pose_array);

    RCLCPP_INFO(this->get_logger(), "[PlaceablePoseDetection] Object count = %d", object_num);
}



void PlaceableDetectionNode::activate() {
    
    YAML::Node config = YAML::LoadFile(placeable_param_path_);

    pcp_.reset();
    pcp_ = std::make_shared<pcl_object_detection::PointCloudProcessor>("point_cloud_processor_5");


    placeable_search_interval_ = config["placeable_search_interval"].as<double>(0.05);
    obstacle_tolerance_ = config["obstacle_tolerance"].as<double>(0.1);
    use_voxel_ = config["use_voxel"].as<bool>(true);


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
    RCLCPP_INFO(this->get_logger(), "PlaceableDetectionNode activated");
}

void PlaceableDetectionNode::deactivate() {
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
    // RCLCPP_INFO(this->get_logger(), "PlaceableDetectionNode deactivated");
}
