#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_object_detection/point_cloud_processor.hpp>
#include <pcl_object_detection/PCLParameterConfig.h>

#include <sobits_msgs/RunCtrl.h>
#include <sobits_msgs/ObjectPose.h>
#include <sobits_msgs/ObjectPoseArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

namespace pcl_object_detection {
    class PlaceablePoseDetection : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Subscriber sub_point_cloud_;

            ros::Publisher pub_cloud_detection_range_;
            ros::Publisher pub_cloud_object_;
            ros::Publisher pub_pose_array_;

            ros::ServiceServer srv_subscriber_switch_;

            tf2_ros::TransformBroadcaster broadcaster_;

            std::string pointcloud_topic_;
            std::string target_frame_;
            bool use_sobit_pro_;
            bool need_cloud_detection_range_;
            bool need_pose_array_;
            bool need_cloud_object_;
            bool need_tf_;
            bool use_voxel_;
            double placeable_search_interval_;
            double obstacle_tolerance_;

            std::unique_ptr<pcl_object_detection::PointCloudProcessor> pcp_;

            dynamic_reconfigure::Server<pcl_object_detection::PCLParameterConfig>* server_;
            dynamic_reconfigure::Server<pcl_object_detection::PCLParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(pcl_object_detection::PCLParameterConfig& config, uint32_t level);
            bool callbackSubscriberSwitch( sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res  );
            void callbackCloud( const sensor_msgs::PointCloud2ConstPtr& cloud_msg );

        public:
            virtual void onInit();

    };
}

void pcl_object_detection::PlaceablePoseDetection::callbackDynamicReconfigure(pcl_object_detection::PCLParameterConfig& config, uint32_t level) {
    pointcloud_topic_ = config.pointcloud_topic_name;
    target_frame_ = config.base_frame_name;
    need_cloud_detection_range_ = config.publish_cloud_detection_range;
    need_cloud_object_ = config.publish_cloud_object;
    need_pose_array_ = config.publish_pose_array;
    use_voxel_ = config.use_voxel;
    placeable_search_interval_ = config.placeable_search_interval;
    obstacle_tolerance_ = config.obstacle_tolerance;

    pcp_->setTargetFrame( config.base_frame_name );
    pcp_->setFlag( config.use_tf );
    pcp_->setPassThroughParameters( config.passthrough_x_min, config.passthrough_x_max, config.passthrough_y_min, config.passthrough_y_max, config.passthrough_z_min, config.passthrough_z_max );
    pcp_->setVoxelGridParameter( config.leaf_size );
    pcp_->setClusteringParameters( config.cluster_tolerance, config.min_cluster_point_size, config.max_cluster_point_size  );
    pcp_->setSACSegmentationParameter( pcl::SACMODEL_PERPENDICULAR_PLANE, pcl::SAC_RANSAC, config.threshold_distance, config.probability );
    pcp_->setObjectSizeParameter( config.object_size_x_min, config.object_size_x_max, config.object_size_y_min, config.object_size_y_max, config.object_size_z_min, config.object_size_z_max );
    pcp_->setObjectOffsetParameter( config.object_centroid_offset_x, config.object_centroid_offset_y, config.object_centroid_offset_z );
    return;
}

bool pcl_object_detection::PlaceablePoseDetection::callbackSubscriberSwitch( sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res ) {
    if ( req.request ) {
        NODELET_INFO ("[ PlaceablePoseDetection ] Turn on the PlaceablePoseDetection" );
        sub_point_cloud_ = nh_.subscribe(pointcloud_topic_, 10, &PlaceablePoseDetection::callbackCloud, this); //オン（再定義）
    } else {
        NODELET_INFO ("[ PlaceablePoseDetection ] Turn off the PlaceablePoseDetection" );
        sub_point_cloud_.shutdown();//オフ
    }
    res.response = true;
    return true;
}

void pcl_object_detection::PlaceablePoseDetection::callbackCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    PointCloud::Ptr cloud (new PointCloud());
    PointCloud::Ptr cloud_plane (new PointCloud());
    PointCloud::Ptr cloud_plane_hull (new PointCloud());
    sobits_msgs::ObjectPoseArrayPtr pose_array (new sobits_msgs::ObjectPoseArray);
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
    // 平面より下の物体点群を除去
    Eigen::Vector4f centroid,  min_pt, max_pt;
    pcl::compute3DCentroid( *cloud_plane, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z(), centroid.z()+0.4 );
    pcp_->passThrough( cloud, cloud );
    // 平面より奥の物体を削除
    pcl::getMinMax3D( *cloud_plane, min_pt, max_pt);
    if ( use_sobit_pro_ ) {
        pcp_->setPassThroughParameters( "y", 0.0, max_pt.y() );
        pcp_->passThrough( cloud, cloud );
    } else {
        pcp_->setPassThroughParameters( "x", 0.0, max_pt.x() );
        pcp_->passThrough( cloud, cloud );
    }

    // 物体の数を確認
    pcp_->euclideanClusterExtraction ( cloud, &cluster_indices );
    int object_num = cluster_indices.size();

    // 平面の端を取得し、物体点群を追加
    pcp_->voxelGrid( cloud, cloud );
    pcp_->ConcaveHull( cloud_plane, cloud_plane_hull );
    *cloud = *cloud + *cloud_plane_hull;

    // 配置位置の推定範囲を決定
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
    geometry_msgs::Point placeable_point;
    double min_pot = 1.0, potential = 0.0;

    // 配置位置推定
    geometry_msgs::Point obs_pt;
    double placeable_search_interval = placeable_search_interval_;
    double obstacle_tolerance = obstacle_tolerance_;
    for ( double x = max_pt.x() - 0.05; x > min_pt.x() + 0.05; x -= placeable_search_interval ) {
        for ( double y = max_pt.y() - 0.05; y >min_pt.y() + 0.05; y -= placeable_search_interval ) {
            geometry_msgs::Point search_pt;
            pcl::PointIndices::Ptr nearest_inliers (new pcl::PointIndices);
            search_pt.x = x;
            search_pt.y = y;
            search_pt.z = centroid.z();
            if ( !pcp_->nearestKSearch ( cloud, nearest_inliers, search_pt )) continue;
            obs_pt.x = cloud->points[ nearest_inliers->indices[0] ].x;
            obs_pt.y = cloud->points[ nearest_inliers->indices[0] ].y;
            double obs_dist = std::hypotf( search_pt.x - obs_pt.x, search_pt.y - obs_pt.y );
            if ( obs_dist < obstacle_tolerance ) potential = 1.0;
            else potential = ( 1 / ( 1 + obs_dist ));

            if ( min_pot > potential ) {
                min_pot = potential;
                placeable_point = search_pt;
            }
        }
    }

    if ( min_pot != 1.0 ) {
        sobits_msgs::ObjectPose pose;
        pose.Class = "placeable_point";
        pose.pose.position = placeable_point;
        if ( use_sobit_pro_ ) {
            sobits_msgs::ObjectPose pose_pro;
            pose_pro.pose.position.x = pose.pose.position.x + 0.221;
            pose_pro.pose.position.y = pose.pose.position.y - 0.221;
            pose_pro.pose.position.z = pose.pose.position.z;
            pose_array->object_poses.push_back(pose_pro);
        } else pose_array->object_poses.push_back(pose);

        // tf::Transform transform;
        // transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
        // tf::Quaternion q;
        tf2::Transform transform;
        transform.setOrigin( tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
        tf2::Quaternion q;
        geometry_msgs :: TransformStamped transformStamped;

        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = target_frame_;
        transformStamped.child_frame_id = "placeable_point";
        transformStamped.transform.translation.x = transform.getOrigin().x();
        transformStamped.transform.translation.y = transform.getOrigin().y();
        transformStamped.transform.translation.z = transform.getOrigin().z();
        transformStamped.transform.rotation = tf2::toMsg(transform.getRotation());

        broadcaster_.sendTransform(transformStamped);

    } else ROS_ERROR("NO placeable_point");

    cloud_plane->header.frame_id = target_frame_;
    cloud->header.frame_id = target_frame_;
    pcl_conversions::toPCL(ros::Time::now(), cloud_plane->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    if ( need_cloud_detection_range_ ) pub_cloud_detection_range_.publish( cloud_plane );
    if ( need_cloud_object_ ) pub_cloud_object_.publish( cloud );
    if ( need_pose_array_ ) pub_pose_array_.publish( pose_array );
    NODELET_INFO("\033[1;32m[ PlaceablePoseDetection ]\tObject count = %d \033[m", object_num );
    return;
}

void pcl_object_detection::PlaceablePoseDetection::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pcp_.reset( new pcl_object_detection::PointCloudProcessor );

    srv_subscriber_switch_ = nh_.advertiseService( "placeable_detection_run_ctr", &PlaceablePoseDetection::callbackSubscriberSwitch, this);

    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<pcl_object_detection::PCLParameterConfig>(pnh_);
    f_ = boost::bind(&PlaceablePoseDetection::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    pub_cloud_detection_range_ = nh_.advertise<PointCloud>("cloud_detection_range", 1);
    pub_cloud_object_ = nh_.advertise<PointCloud>("cloud_object", 1);
    pub_pose_array_ = nh_.advertise<sobits_msgs::ObjectPoseArray>("object_poses", 1);

}

PLUGINLIB_EXPORT_CLASS(pcl_object_detection::PlaceablePoseDetection, nodelet::Nodelet)
