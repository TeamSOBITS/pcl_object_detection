

#include <pcl_object_detection/point_cloud_processor.hpp>

using namespace pcl_object_detection;

PointCloudProcessor::PointCloudProcessor(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), tfBroadcaster_(nd) {
    // global parameter //
    base_frame_name_ = nd_->get_parameter("base_frame_name").as_string();
    publish_cloud_detection_range_ = nd_->get_parameter("publish_cloud_detection_range").as_bool();
    publish_cloud_object_ = nd_->get_parameter("publish_cloud_object").as_bool();
    publish_pose_array_ = nd_->get_parameter("publish_pose_array").as_bool();
    use_tf_ = nd_->get_parameter("use_tf").as_bool();

    use_voxel_ = nd_->get_parameter("use_voxel").as_bool();
    leaf_size_ = nd_->get_parameter("leaf_size").as_double();

    cluster_tolerance_ = nd_->get_parameter("cluster_tolerance").as_double();
    min_cluster_point_size_ = nd_->get_parameter("min_cluster_point_size").as_int();
    max_cluster_point_size_ = nd_->get_parameter("max_cluster_point_size").as_int();

    threshold_distance_ = nd_->get_parameter("threshold_distance").as_double();
    probability_ = nd_->get_parameter("probability").as_double();

    object_size_x_min_ = nd_->get_parameter("object_size_x_min").as_double();
    object_size_x_max_ = nd_->get_parameter("object_size_x_max").as_double();
    object_size_y_min_ = nd_->get_parameter("object_size_y_min").as_double();
    object_size_y_max_ = nd_->get_parameter("object_size_y_max").as_double();
    object_size_z_min_ = nd_->get_parameter("object_size_z_min").as_double();
    object_size_z_max_ = nd_->get_parameter("object_size_z_max").as_double();
    // global parameter //

    setSACSegmentationParameter(pcl::SACMODEL_PERPENDICULAR_PLANE, pcl::SAC_RANSAC);
    setRadiusOutlierRemovalParameters( 0.05, 20, false );
    setClusteringParameters();
    setVoxelGridParameter();

    tree_.reset(new pcl::search::KdTree<PointT>());
}


bool PointCloudProcessor::transformFramePointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud,
    PointCloud::Ptr output_cloud) {
    // Transform lookup
    pcl::fromROSMsg(*input_cloud, *output_cloud);

    if (!tfBuffer_.canTransform(base_frame_name_, input_cloud->header.frame_id, rclcpp::Time(0), std::chrono::milliseconds(500))) {
        RCLCPP_WARN(nd_->get_logger(), "Waiting for transform from %s to %s...",
                    input_cloud->header.frame_id.c_str(), base_frame_name_.c_str());
        return false;
    }    

    auto transform_stamped = tfBuffer_.lookupTransform(
        base_frame_name_, input_cloud->header.frame_id, tf2::TimePointZero);

    // Convert to Eigen Matrix
    Eigen::Isometry3d transform_iso = tf2::transformToEigen(transform_stamped.transform);
    Eigen::Matrix4f transform_matrix = transform_iso.matrix().cast<float>();

    // Transform the point cloud
    pcl::transformPointCloud(*output_cloud, *output_cloud, transform_matrix);

    return true;
}


void pcl_object_detection::PointCloudProcessor::setSACSegmentationParameter(const int model,  const int method) {
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType (model);
    seg_.setMethodType (method);
    seg_.setDistanceThreshold (threshold_distance_);
    seg_.setProbability(probability_);
    seg_.setMaxIterations(1000);
}


bool PointCloudProcessor::transformFrameScan2D2PointCloud(const sensor_msgs::msg::LaserScan::SharedPtr &input_scan2d, PointCloud::Ptr output_cloud) {
    sensor_msgs::msg::PointCloud2 cloud;
    tf2::TimePoint scan_time = tf2_ros::fromMsg(input_scan2d->header.stamp); // ROSメッセージのTimeをtf2::TimePointに変換


    if (!base_frame_name_.empty()) {


        if (!tfBuffer_.canTransform(base_frame_name_, input_scan2d->header.frame_id, scan_time, std::chrono::milliseconds(500))) {
            RCLCPP_WARN(nd_->get_logger(), "Waiting for transform from %s to %s...",
                        input_scan2d->header.frame_id.c_str(), base_frame_name_.c_str());
            return false;
        }
        
        try {
            geometry_msgs::msg::TransformStamped transform =
                tfBuffer_.lookupTransform(base_frame_name_, input_scan2d->header.frame_id, tf2::TimePointZero);
            projector_.transformLaserScanToPointCloud(base_frame_name_, *input_scan2d, cloud, tfBuffer_);
            pcl::fromROSMsg(cloud, *output_cloud);
            output_cloud->header.frame_id = base_frame_name_;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
            return false;
        }
    } else {
        RCLCPP_ERROR(nd_->get_logger(), "Please set the target frame.");
        return false;
    }
    return true;
}


// geometry_msgs::msg::Point PointCloudProcessor::transformPoint(const std::string &org_frame, const std::string &target_frame, const geometry_msgs::msg::Point &point) {
//     geometry_msgs::msg::PointStamped pt_transformed;
//     geometry_msgs::msg::PointStamped pt;
//     pt.header.frame_id = org_frame;
//     pt.header.stamp = this->get_clock()->now();
//     pt.point = point;

//     try {
//         pt_transformed = tfBuffer_.transform(pt, target_frame);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
//     }

//     return pt_transformed.point;
// }


void pcl_object_detection::PointCloudProcessor::setVoxelGridParameter() {
    voxel_.setLeafSize( leaf_size_, leaf_size_, leaf_size_ );
}


bool pcl_object_detection::PointCloudProcessor::passThrough(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud) {
    try {
        pass_.setInputCloud(input_cloud);
        pass_.filter(*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}


void pcl_object_detection::PointCloudProcessor::setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
    pass_.setFilterFieldName( axis );
    pass_.setFilterLimits( limit_min, limit_max);
}

void pcl_object_detection::PointCloudProcessor::passThroughXYZ(PointCloud::Ptr cloud, 
                                        double x_min, double x_max, 
                                        double y_min, double y_max, 
                                        double z_min, double z_max) {
    setPassThroughParameters("x", x_min, x_max);
    passThrough(cloud, cloud);
    setPassThroughParameters("y", y_min, y_max);
    passThrough(cloud, cloud);
    setPassThroughParameters("z", z_min, z_max);
    passThrough(cloud, cloud);
}

void pcl_object_detection::PointCloudProcessor::setSACPlaneParameter( const std::string &axis, const double eps_angle_degree ) {
    Eigen::Vector3f axis_vec;
    if      ( axis == "x" ) axis_vec = Eigen::Vector3f(1.0,0.0,0.0); //y axis
    else if ( axis == "y" ) axis_vec = Eigen::Vector3f(0.0,1.0,0.0); //y axis
    else if ( axis == "z" ) axis_vec = Eigen::Vector3f(0.0,0.0,1.0); //y axis
    else return;
    seg_.setAxis(axis_vec);
    seg_.setEpsAngle( eps_angle_degree * (M_PI/180.0f) ); // plane can be within eps_angle_degree degrees of plane
}

void pcl_object_detection::PointCloudProcessor::setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized ) {
    outrem_.setRadiusSearch( radius );
    outrem_.setMinNeighborsInRadius ( min_pts );
    outrem_.setKeepOrganized( keep_organized );
}

bool pcl_object_detection::PointCloudProcessor::voxelGrid(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud) {
    if (!use_voxel_) return true;
    try {
        voxel_.setInputCloud(input_cloud);
        voxel_.filter(*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}

bool pcl_object_detection::PointCloudProcessor::euclideanClusterExtraction(const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices> *output_indices) {
    try {
        tree_->setInputCloud(input_cloud);
        ec_.setInputCloud(input_cloud);
        ec_.extract(*output_indices);
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}

void pcl_object_detection::PointCloudProcessor::setClusteringParameters () {
    ec_.setClusterTolerance( cluster_tolerance_ );
    ec_.setMinClusterSize( min_cluster_point_size_ );
    ec_.setMaxClusterSize( max_cluster_point_size_ );
    ec_.setSearchMethod( tree_ );
}

bool pcl_object_detection::PointCloudProcessor::extractIndices(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative) {
    try {
        extract_.setInputCloud(input_cloud);
        extract_.setIndices(indices);
        extract_.setNegative(negative);
        extract_.filter(*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}

bool pcl_object_detection::PointCloudProcessor::radiusOutlierRemoval(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud) {
    try {
        outrem_.setInputCloud(input_cloud);
        outrem_.filter(*output_cloud);
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}

bool pcl_object_detection::PointCloudProcessor::sacSegmentation(const PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) {
    try {
        seg_.setInputCloud(input_cloud);
        seg_.segment(*inliers, *coefficients);
        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(nd_->get_logger(), "%s", ex.what());
        return false;
    }
}

// // radius Search :
// bool PointCloudProcessor::radiusSearch ( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::msg::Point& search_pt, const double radius, bool is_accept_add_point ) {
//     try{
//         bool is_match = false;
//         PointT searchPoint;
//         searchPoint .x = search_pt .x;
//         searchPoint .y = search_pt .y;
//         searchPoint .z = search_pt.z;
//         std::vector<int> pointIdxRadiusSearch;
//         std::vector<float> pointRadiusSquaredDistance;
//         flann_ .setInputCloud ( input_cloud );

//         if ( flann_.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0) > 0 ) {
//             unsigned int size = pointIdxRadiusSearch.size();
//             for ( unsigned int i = 0; i < size; ++i ) { output_indices ->indices .push_back ( pointIdxRadiusSearch [i] ); }
//             is_match = true;
//         }
//         if ( !is_match && is_accept_add_point ) {
//             input_cloud->points.push_back(searchPoint);
//             output_indices ->indices .push_back ( input_cloud->points.size() - 1 );
//             is_match = true;
//         }
//         return is_match;
//     } catch ( std::exception& ex ) {
//         RCLCPP_ERROR(nd_->get_logger(),"%s", ex.what());
//         return false;
//     }
// }

bool pcl_object_detection::PointCloudProcessor::nearestKSearch( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::msg::Point& search_pt, const int K ) {
    try{
        bool is_match = false;
        PointT searchPoint;
        searchPoint .x = search_pt .x;
        searchPoint .y = search_pt .y;
        searchPoint .z = search_pt .z;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        flann_.setInputCloud ( input_cloud );
        if ( flann_.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            unsigned int size = pointIdxNKNSearch.size();
            for ( unsigned int i = 0; i < size; ++i ) { output_indices ->indices .push_back ( pointIdxNKNSearch [i] ); }
            is_match = true;
        }
        return is_match;
    } catch ( std::exception& ex ) {
        RCLCPP_ERROR(nd_->get_logger(),"%s", ex.what());
        return false;
    }
}

bool pcl_object_detection::PointCloudProcessor::ConcaveHull( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) {
    if (input_cloud->width > 0){
        try{
            hull_.setInputCloud(input_cloud);
            hull_.setAlpha(0.03);
            hull_.reconstruct(*output_cloud);
            output_cloud->header.frame_id = input_cloud->header.frame_id;
            return true;
        } catch ( std::exception& ex ) {
            RCLCPP_ERROR(nd_->get_logger(),"%s", ex.what());
            return false;
        }
    }
    else{
        return false;
    }
}

bool pcl_object_detection::PointCloudProcessor::compareDistance(vision_msgs::msg::Detection3D &a, vision_msgs::msg::Detection3D &b) {
    double a_dist = std::hypotf( a.results[0].pose.pose.position.x,  a.results[0].pose.pose.position.y );
    double b_dist = std::hypotf( b.results[0].pose.pose.position.x,  b.results[0].pose.pose.position.y );
    return a_dist < b_dist; //近い順
}

void pcl_object_detection::PointCloudProcessor::sendTransform(const geometry_msgs::msg::Pose target_pose, const std::string &target_frame) {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = nd_->now();
    transformStamped.header.frame_id = base_frame_name_;
    transformStamped.child_frame_id = target_frame;
    transformStamped.transform.translation.x = target_pose.position.x;
    transformStamped.transform.translation.y = target_pose.position.y;
    transformStamped.transform.translation.z = target_pose.position.z;
    transformStamped.transform.rotation.x = target_pose.orientation.x;
    transformStamped.transform.rotation.y = target_pose.orientation.y;
    transformStamped.transform.rotation.z = target_pose.orientation.z;
    transformStamped.transform.rotation.w = target_pose.orientation.w;

    tfBroadcaster_.sendTransform(transformStamped);
}

int PointCloudProcessor::principalComponentAnalysis(
    const PointCloud::Ptr cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    vision_msgs::msg::Detection3DArray::SharedPtr pose_array_msg,
    PointCloud::Ptr cloud_object,
    const int init_object_id )
{
    for ( auto& cluster : cluster_indices ) {
        Eigen::Vector4f pca_centroid;
        pcl::compute3DCentroid( *cloud, cluster, pca_centroid );
        // Ref : https://programmersought.com/article/88204491934/
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pca_centroid, covariance); // Computes the normalized 3x3 covariance matrix (variance-covariance matrix)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors); // Find eigenvalues and eigenvectors
        Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors(); // eigenvector
        //Eigen::Vector3f eigen_values_pca = eigen_solver.eigenvalues();   // eigenvalue

        Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
        transform.block<3, 3>(0, 0) = eigen_vectors_pca.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pca_centroid.head<3>());//

        PointCloud::Ptr cloud_transformed(new PointCloud() );
        pcl::transformPointCloud(*cloud, cluster, *cloud_transformed, transform); // Rotation with eigenvectors
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D( *cloud_transformed, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if ( cluster_size.x() < object_size_x_min_ || cluster_size.x() > object_size_x_max_ ) continue;
        if ( cluster_size.y() < object_size_y_min_ || cluster_size.y() > object_size_y_max_ ) continue;
        if ( cluster_size.z() < object_size_z_min_ || cluster_size.z() > object_size_z_max_ ) continue;
        for ( auto& i : cluster.indices ) cloud_object->points.push_back(cloud->points[i]);

        // double roll = std::atan2( eigen_vectors_pca(1, 0), eigen_vectors_pca(2, 0) );
        // double pitch = std::atan2( eigen_vectors_pca(2, 0), eigen_vectors_pca(0, 0) );
        double yaw = std::atan2( eigen_vectors_pca(1, 0), eigen_vectors_pca(0, 0) ) + M_PI;

        // sobits_interfaces::msg::ObjectPose pose;
        vision_msgs::msg::Detection3D pose;
        vision_msgs::msg::ObjectHypothesisWithPose ohwp;
        // ohwp.hypothesis.class_id = "OBJECT_TF";
        ohwp.hypothesis.score = 1.0;
        ohwp.pose.pose.position.x = pca_centroid(0);
        ohwp.pose.pose.position.y = pca_centroid(1);
        ohwp.pose.pose.position.z = pca_centroid(2);
        ohwp.pose.pose.orientation.x = 0.;
        ohwp.pose.pose.orientation.y = 0.;
        ohwp.pose.pose.orientation.z = sin(yaw / 2);
        ohwp.pose.pose.orientation.w = cos(yaw / 2);
        pose.header.stamp = nd_->now();
        pose.header.frame_id = base_frame_name_;
        pose.results.push_back(ohwp);
        pose.bbox.center.position.x = pca_centroid(0);
        pose.bbox.center.position.y = pca_centroid(1);
        pose.bbox.center.position.z = pca_centroid(2);
        pose.bbox.center.orientation.x = 0.;
        pose.bbox.center.orientation.y = 0.;
        pose.bbox.center.orientation.z = sin(yaw / 2);
        pose.bbox.center.orientation.w = cos(yaw / 2);
        pose.bbox.size.x = cluster_size.x();
        pose.bbox.size.y = cluster_size.y();
        pose.bbox.size.z = cluster_size.z();
        // pose.id = "OBJECT_TF";
        pose_array_msg->detections.push_back(pose);
    }
    std::sort(pose_array_msg->detections.begin(), pose_array_msg->detections.end(), compareDistance);

    int object_id = init_object_id;
    for ( auto& pose : pose_array_msg->detections ) {
        pose.id                             = "object_" + std::to_string(object_id);
        pose.results[0].hypothesis.class_id = "object_" + std::to_string(object_id);
        sendTransform(pose.bbox.center, "object_" + std::to_string(object_id));
        object_id++;
    }

    cloud_object->header.frame_id = cloud->header.frame_id;
    pose_array_msg->header.frame_id = cloud->header.frame_id;
    pose_array_msg->header.stamp = nd_->now();
    pcl_conversions::toPCL(nd_->now(), cloud_object->header.stamp);
    return  ( object_id == init_object_id ) ? -1 : object_id;
}

