#include <pcl_object_detection/point_cloud_processor.hpp>

using namespace pcl_object_detection;

PointCloudProcessor::PointCloudProcessor() {
    setPassThroughParameters( 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 );
    setPassThroughParameters( "z", 0.1, 1.0);
    setVoxelGridParameter( 0.01 );
    setClusteringParameters( 0.05, 1, 1000 );
    setSACSegmentationParameter( pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.01, 0.95 );
    setRadiusOutlierRemovalParameters( 0.05, 20, false );
    setObjectSizeParameter( 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 );
    setObjectOffsetParameter( 0.0, 0.0, 0.0 );
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}
bool PointCloudProcessor::transformFramePointCloud ( const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ) {
    PointCloud cloud_src;
    pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
    if (target_frame_.empty() == false ){
        try {
            tf_listener_.waitForTransform(target_frame_, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame_, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
            output_cloud->header.frame_id = target_frame_;
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    } else ROS_ERROR("Please set the target frame.");
    return true;
}
bool PointCloudProcessor::transformFrameScan2D2PointCloud ( const sensor_msgs::LaserScanConstPtr &input_scan2d, PointCloud::Ptr output_cloud ) {
    sensor_msgs::PointCloud2Ptr cloud ( new sensor_msgs::PointCloud2 );
    if (target_frame_.empty() == false ) {
        tf_listener_.waitForTransform(target_frame_, input_scan2d->header.frame_id, input_scan2d->header.stamp, ros::Duration(5.0));
        projector_.transformLaserScanToPointCloud(target_frame_, *input_scan2d, *cloud, tf_listener_);
        pcl::fromROSMsg<PointT>(*cloud, *output_cloud);
        output_cloud->header.frame_id = target_frame_;
    } else ROS_ERROR("Please set the target frame.");
    return true;
}

geometry_msgs::Point PointCloudProcessor::transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
    geometry_msgs::PointStamped pt_transformed;
    geometry_msgs::PointStamped pt;
    pt.header.frame_id = org_frame;
    pt.header.stamp = ros::Time(0);
    pt.point = point;
    if ( tf_listener_.frameExists( target_frame ) ) {
        try {
            tf_listener_.transformPoint( target_frame, pt, pt_transformed );
        } catch ( const tf::TransformException& ex ) {
            ROS_ERROR( "%s",ex.what( ) );
        }
    } else {
        ROS_ERROR("target_frame is not Exists");
    }
    return pt_transformed.point;
}

// passThrough a point cloud :
bool PointCloudProcessor::passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        pass_.setInputCloud( input_cloud );
        pass_.filter( *output_cloud );
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
void PointCloudProcessor::passThroughXYZ( PointCloud::Ptr cloud ) {
    setPassThroughParameters( "x", pass_param.x_min, pass_param.x_max );
    passThrough( cloud, cloud );
    setPassThroughParameters( "y", pass_param.y_min, pass_param.y_max );
    passThrough( cloud, cloud );
    setPassThroughParameters( "z", pass_param.z_min, pass_param.z_max );
    passThrough( cloud, cloud );
}

// Downsample a point cloud :
bool PointCloudProcessor::voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        voxel_.setInputCloud( input_cloud );
        voxel_.filter( *output_cloud );
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// clustering extraction :
bool PointCloudProcessor::euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
    try {
        tree_->setInputCloud( input_cloud );
        ec_.setInputCloud( input_cloud );
        ec_.extract( *output_indices );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Extract specified Indices from point cloud :
bool PointCloudProcessor::extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative ) {
    try {
        extract_.setInputCloud( input_cloud );
        extract_.setIndices( indices );
        extract_.setNegative( negative );
        extract_.filter( *output_cloud );
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

// Removing outliers using a RadiusOutlier removal :
bool PointCloudProcessor::radiusOutlierRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try {
        outrem_.setInputCloud( input_cloud );
        outrem_.filter (*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Random Sample Consensus model(https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html)
bool PointCloudProcessor::sacSegmentation( const PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients ) {
    try{
        seg_.setInputCloud (input_cloud);
        seg_.segment (*inliers, *coefficients);
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

// radius Search :
bool PointCloudProcessor::radiusSearch ( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::Point& search_pt, const double radius, bool is_accept_add_point ) {
    try{
        bool is_match = false;
        PointT searchPoint;
        searchPoint .x = search_pt .x;
        searchPoint .y = search_pt .y;
        searchPoint .z = search_pt.z;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        flann_ .setInputCloud ( input_cloud );

        if ( flann_.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0) > 0 ) {
            unsigned int size = pointIdxRadiusSearch.size();
            for ( unsigned int i = 0; i < size; ++i ) { output_indices ->indices .push_back ( pointIdxRadiusSearch [i] ); }
            is_match = true;
        }
        if ( !is_match && is_accept_add_point ) {
            input_cloud->points.push_back(searchPoint);
            output_indices ->indices .push_back ( input_cloud->points.size() - 1 );
            is_match = true;
        }
        return is_match;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool PointCloudProcessor::nearestKSearch( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::Point& search_pt, const int K ) { 
    try{
        bool is_match = false;
        PointT searchPoint;
        searchPoint .x = search_pt .x;
        searchPoint .y = search_pt .y;
        searchPoint .z = search_pt .z;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        flann_ .setInputCloud ( input_cloud );
        if ( flann_.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            unsigned int size = pointIdxNKNSearch.size();
            for ( unsigned int i = 0; i < size; ++i ) { output_indices ->indices .push_back ( pointIdxNKNSearch [i] ); }
            is_match = true;
        }
        return is_match;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool PointCloudProcessor::ConcaveHull( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    try{
        hull_.setInputCloud(input_cloud);
        hull_.setAlpha(0.03);
        hull_.reconstruct(*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

int PointCloudProcessor::principalComponentAnalysis(
    const PointCloud::Ptr cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    sobit_common_msg::ObjectPoseArrayPtr pose_array_msg,
    PointCloud::Ptr cloud_object,
    const int init_object_id )
{
    ObjectSizeParameter obj_size = obj_param;
    std::string target_frame = target_frame_;
    bool need_tf = need_tf_;
    for ( auto& cluster : cluster_indices ) {
        Eigen::Vector4f pca_centroid;
        pcl::compute3DCentroid( *cloud, cluster, pca_centroid );
        // Ref : https://programmersought.com/article/88204491934/
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pca_centroid, covariance); // 正規化された3x3共分散行列(分散共分散行列)を計算
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors); // 固有値と固有ベクトルを求める
        Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors(); // 固有ベクトル
        //Eigen::Vector3f eigen_values_pca = eigen_solver.eigenvalues();   // 固有値

        Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
        transform.block<3, 3>(0, 0) = eigen_vectors_pca.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pca_centroid.head<3>());//

        PointCloud::Ptr cloud_transformed(new PointCloud() );
        pcl::transformPointCloud(*cloud, cluster, *cloud_transformed, transform); // 固有ベクトルで回転
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D( *cloud_transformed, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if ( cluster_size.x() < obj_size.x_min || cluster_size.x() > obj_size.x_max ) continue;
        if ( cluster_size.y() < obj_size.y_min || cluster_size.y() > obj_size.y_max ) continue;
        if ( cluster_size.z() < obj_size.z_min || cluster_size.z() > obj_size.z_max ) continue;
        for ( auto& i : cluster.indices ) cloud_object->points.push_back(cloud->points[i]);

        // double roll = std::atan2( eigen_vectors_pca(1, 0), eigen_vectors_pca(2, 0) );
        // double pitch = std::atan2( eigen_vectors_pca(2, 0), eigen_vectors_pca(0, 0) );
        double yaw = std::atan2( eigen_vectors_pca(1, 0), eigen_vectors_pca(0, 0) ) + M_PI;

        sobit_common_msg::ObjectPose pose;
        pose.pose.position.x = pca_centroid(0)+obj_size.x_offset;
        pose.pose.position.y = pca_centroid(1)+obj_size.y_offset;
        pose.pose.position.z = pca_centroid(2)+obj_size.z_offset;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);
        pose_array_msg->object_poses.push_back(pose);
    }
    std::sort(pose_array_msg->object_poses.begin(), pose_array_msg->object_poses.end(), PointCloudProcessor::compareDistance);

    int object_id = init_object_id;
    for ( auto& pose : pose_array_msg->object_poses ) {
        pose.Class = "object_" + std::to_string(object_id);
        pose.detect_id = object_id;
        if ( need_tf ) {
            tf::Transform tf_transform;
            tf_transform.setOrigin( tf::Vector3( pose.pose.position.x, pose.pose.position.y, pose.pose.position.z ) );
            tf::Quaternion quat_tf;
            quaternionMsgToTF(pose.pose.orientation , quat_tf);
            tf_transform.setRotation( quat_tf );
            broadcaster_.sendTransform( tf::StampedTransform ( tf_transform, ros::Time::now(), target_frame_, pose.Class ));
        }
        object_id++;
    }

    cloud_object->header.frame_id = cloud->header.frame_id;
    pose_array_msg->header.frame_id = cloud->header.frame_id;
    pose_array_msg->header.stamp = ros::Time::now();
    pcl_conversions::toPCL(ros::Time::now(), cloud_object->header.stamp);
    return  ( object_id == init_object_id ) ? -1 : object_id;
}
