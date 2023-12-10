#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_object_detection/point_cloud_processor.hpp>
#include <pcl_object_detection/LineDetectionParameterConfig.h>
#include <pcl_object_detection/LineInfo.h>
#include <pcl_object_detection/LineDetectionService.h>

#include <pcl_object_detection/RunCtrl.h>
#include <pcl_object_detection/ObjectPose.h>
#include <pcl_object_detection/ObjectPoseArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

namespace pcl_object_detection {
    class LineDetection : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Subscriber sub_scan_;

            ros::Publisher pub_cloud_line_;
            ros::Publisher pub_marker_;
            ros::Publisher pub_line_info_;

            ros::ServiceServer srv_subscriber_switch_;
            ros::ServiceServer srv_line_detection_;

            std::string scan_topic_;
            std::string target_frame_;
            bool need_marker_;
            bool need_cloud_line_;
            bool need_line_info_;
            double angle_deg_;
            double distance_;

            std::unique_ptr<pcl_object_detection::PointCloudProcessor> pcp_;

            dynamic_reconfigure::Server<pcl_object_detection::LineDetectionParameterConfig>* server_;
            dynamic_reconfigure::Server<pcl_object_detection::LineDetectionParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(pcl_object_detection::LineDetectionParameterConfig& config, uint32_t level);
            bool callbackSubscriberSwitch( pcl_object_detection::RunCtrl::Request &req, pcl_object_detection::RunCtrl::Response &res  );
            void callbackScan2D ( const sensor_msgs::LaserScanConstPtr &scan2d_msg );
            bool detectLine( pcl_object_detection::LineDetectionService::Request &req, pcl_object_detection::LineDetectionService::Response &res );
            visualization_msgs::Marker makeMakerString( const std::string string, const double x, const double y, const double z );

        public:
            virtual void onInit();

    };
}

void pcl_object_detection::LineDetection::callbackDynamicReconfigure(pcl_object_detection::LineDetectionParameterConfig& config, uint32_t level) {
    scan_topic_ = config.scan_topic_name;
    target_frame_ = config.base_frame_name;
    need_marker_ = config.publish_marker;
    need_cloud_line_ = config.publish_cloud_line;
    need_line_info_ = config.publish_line_info;

    if ( config.run_ctr ) {
        NODELET_INFO ("[ ObjectDetectionFloor ] Turn on the LineDetection" );
        sub_scan_ = nh_.subscribe(scan_topic_, 10, &LineDetection::callbackScan2D, this); //オン（再定義）
    } else {
        NODELET_INFO ("[ LineDetection ] Turn off the LineDetection" );
        sub_scan_.shutdown();//オフ
    }
    pcp_->setTargetFrame( config.base_frame_name );
    pcp_->setPassThroughParameters( "y", config.passthrough_y_min, config.passthrough_y_max );
    pcp_->setSACSegmentationParameter(  pcl::SACMODEL_LINE, pcl::SAC_RANSAC, config.threshold_distance, config.probability );
    return;
}

bool pcl_object_detection::LineDetection::callbackSubscriberSwitch( pcl_object_detection::RunCtrl::Request &req, pcl_object_detection::RunCtrl::Response &res ) {
    if ( req.request ) {
        NODELET_INFO ("[ ObjectDetectionFloor ] Turn on the LineDetection" );
        sub_scan_ = nh_.subscribe(scan_topic_, 10, &LineDetection::callbackScan2D, this); //オン（再定義）
    } else {
        NODELET_INFO ("[ LineDetection ] Turn off the LineDetection" );
        sub_scan_.shutdown();//オフ
    }
    res.response = true;
    return true;
}

void pcl_object_detection::LineDetection::callbackScan2D ( const sensor_msgs::LaserScanConstPtr &scan2d_msg ) {
    PointCloud::Ptr cloud_scan2d (new PointCloud());
    PointCloud::Ptr cloud_line( new PointCloud() );
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std_msgs::Float64Ptr line_angle_deg(new std_msgs::Float64);
    pcl_object_detection::LineInfoPtr info (new pcl_object_detection::LineInfo);
    Eigen::Vector4f centroid;

    if ( !pcp_->transformFrameScan2D2PointCloud( scan2d_msg, cloud_scan2d ) ) return;
    pcp_->passThrough( cloud_scan2d, cloud_scan2d );

    pcp_->sacSegmentation( cloud_scan2d, inliers, coefficients );
    pcp_->extractIndices( cloud_scan2d, cloud_line, inliers, false );
    pcl::compute3DCentroid( *cloud_line, centroid );

    cloud_line->header.frame_id = cloud_scan2d->header.frame_id;
    angle_deg_ = coefficients->values[3]*(180/M_PI);
    distance_ = std::hypotf( centroid.x(), centroid.y() );
    NODELET_INFO("\033[1;32m[ LineDetection ]\tAngle[deg] = %.2lf,\tDistance[m] = %.2lf \033[m", angle_deg_, distance_ );
    pcl_conversions::toPCL(ros::Time::now(), cloud_line->header.stamp);
    info->line_angle_deg = angle_deg_;
    info->line_distance = distance_;

    if ( need_cloud_line_ ) pub_cloud_line_.publish( cloud_line );
    if ( need_line_info_ ) pub_line_info_.publish(info);
    if ( need_marker_ ){
        std::string str = "Angle[deg] = " + std::to_string(angle_deg_) + "  Distance[m] = " + std::to_string(distance_);
        pub_marker_.publish(makeMakerString(str, coefficients->values[0], coefficients->values[1], coefficients->values[2] ));
    }
    return;
}

bool pcl_object_detection::LineDetection::detectLine( pcl_object_detection::LineDetectionService::Request &req, pcl_object_detection::LineDetectionService::Response &res ) {
    ros::spinOnce();
    res.line_angle_deg = angle_deg_;
    res.line_distance = distance_;
    return true;

}

visualization_msgs::Marker pcl_object_detection::LineDetection::makeMakerString( const std::string string, const double x, const double y, const double z ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "line_info";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 3.0;
    marker.pose.position.y = 3.0;
    marker.pose.position.z = 0.2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.text = string;

    marker.lifetime = ros::Duration(1.0);
    return marker;
}


void pcl_object_detection::LineDetection::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pcp_.reset( new pcl_object_detection::PointCloudProcessor );

    srv_line_detection_ = nh_.advertiseService("line_detection_service", &LineDetection::detectLine, this);
    srv_subscriber_switch_ = nh_.advertiseService( "run_ctr", &LineDetection::callbackSubscriberSwitch, this);

    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<pcl_object_detection::LineDetectionParameterConfig>(pnh_);
    f_ = boost::bind(&LineDetection::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    pub_cloud_line_ = nh_.advertise<PointCloud>("cloud_line", 1);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("line_info_marker", 1);
    pub_line_info_ = nh_.advertise<pcl_object_detection::LineInfo>("line_info", 1);
}

PLUGINLIB_EXPORT_CLASS(pcl_object_detection::LineDetection, nodelet::Nodelet)
