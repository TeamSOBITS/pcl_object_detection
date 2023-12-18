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
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

namespace pcl_object_detection {
    class ObjectDetectionTable : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Subscriber sub_point_cloud_;

            ros::Publisher pub_cloud_detection_range_;
            ros::Publisher pub_cloud_object_;
            ros::Publisher pub_pose_array_;

            ros::ServiceServer srv_subscriber_switch_;

            std::string pointcloud_topic_;
            bool need_cloud_detection_range_;
            bool need_pose_array_;
            bool need_cloud_object_;
            bool use_voxel_;

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

void pcl_object_detection::ObjectDetectionTable::callbackDynamicReconfigure(pcl_object_detection::PCLParameterConfig& config, uint32_t level) {
    pointcloud_topic_ = config.pointcloud_topic_name;
    need_cloud_detection_range_ = config.publish_cloud_detection_range;
    need_cloud_object_ = config.publish_cloud_object;
    need_pose_array_ = config.publish_pose_array;
    use_voxel_ = config.use_voxel;

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

bool pcl_object_detection::ObjectDetectionTable::callbackSubscriberSwitch( sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res ) {
    if ( req.request ) {
        NODELET_INFO ("[ ObjectDetectionTable ] Turn on the ObjectDetectionTable" );
        sub_point_cloud_ = nh_.subscribe(pointcloud_topic_, 10, &ObjectDetectionTable::callbackCloud, this); //オン（再定義）
    } else {
        NODELET_INFO ("[ ObjectDetectionTable ] Turn off the ObjectDetectionTable" );
        sub_point_cloud_.shutdown();//オフ
    }
    res.response = true;
    return true;
}

void pcl_object_detection::ObjectDetectionTable::callbackCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    PointCloud::Ptr cloud (new PointCloud());
    PointCloud::Ptr cloud_object (new PointCloud());
    sobits_msgs::ObjectPoseArrayPtr pose_array (new sobits_msgs::ObjectPoseArray);
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
    pcl::compute3DCentroid( *cloud, *inliers, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    pcp_->passThrough( cloud, cloud );

    pcl::compute3DCentroid( *cloud, *inliers, centroid );
    pcp_->setPassThroughParameters( "z", centroid.z()+0.01, pcp_->pass_param.z_max );
    pcp_->passThrough( cloud, cloud );

    pcp_->radiusOutlierRemoval( cloud, cloud );
    pcp_->euclideanClusterExtraction ( cloud, &cluster_indices );
    object_num = pcp_->principalComponentAnalysis( cloud, cluster_indices, pose_array, cloud_object );
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), cloud_object->header.stamp);
    if ( need_cloud_detection_range_ ) pub_cloud_detection_range_.publish( cloud );
    if ( need_cloud_object_ ) pub_cloud_object_.publish( cloud_object );
    if ( need_pose_array_ ) pub_pose_array_.publish( pose_array );
    NODELET_INFO("\033[1;32m[ ObjectDetectionTable ]\tObject count = %d \033[m", object_num );
    return;
}

void pcl_object_detection::ObjectDetectionTable::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pcp_.reset( new pcl_object_detection::PointCloudProcessor );

    srv_subscriber_switch_ = nh_.advertiseService( "object_detection_table_run_ctr", &ObjectDetectionTable::callbackSubscriberSwitch, this);

    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<pcl_object_detection::PCLParameterConfig>(pnh_);
    f_ = boost::bind(&ObjectDetectionTable::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    pub_cloud_detection_range_ = nh_.advertise<PointCloud>("cloud_detection_range", 1);
    pub_cloud_object_ = nh_.advertise<PointCloud>("cloud_object", 1);
    pub_pose_array_ = nh_.advertise<sobits_msgs::ObjectPoseArray>("object_poses", 1);

}

PLUGINLIB_EXPORT_CLASS(pcl_object_detection::ObjectDetectionTable, nodelet::Nodelet)
