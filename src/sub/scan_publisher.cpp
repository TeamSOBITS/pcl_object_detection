#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ScanPublisherNode
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Timer timer_;
        ros::Publisher pub_cloud_sensor_;
        PointCloud::Ptr cloud_;
        double theta_;
        double delta_theta_;

        void callbackTimer(const ros::TimerEvent& e);
    public:
        ScanPublisherNode();
};

void ScanPublisherNode::callbackTimer(const ros::TimerEvent& e){
    PointCloud::Ptr cloud_transformed ( new PointCloud() );

    theta_ += delta_theta_;
    if ( theta_ > 0.6 || theta_ < -0.6 ) delta_theta_ = -delta_theta_;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, theta_);
    Eigen::Quaternionf rotation(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::Vector3f offset(0.0, 0.0, 0.0);

    //回転
    pcl::transformPointCloud( *cloud_, *cloud_transformed, offset, rotation );

    pcl_conversions::toPCL(ros::Time::now(), cloud_transformed->header.stamp);
    cloud_transformed->header.frame_id = "base_laser_link";
    pub_cloud_sensor_.publish(cloud_transformed);
    return;
}

ScanPublisherNode::ScanPublisherNode() : nh_(), pnh_("~") {
    pub_cloud_sensor_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_laserscan", 1);
    cloud_.reset( new PointCloud() );
    double limit_y = 4.0;
    for ( double y = 0.0; y < limit_y; y += 0.01 ) {
        PointT p;
        p.x = 2.0; p.y = y; p.z = 0.0;
        cloud_->points.push_back(p);
        p.y = -p.y;
        cloud_->points.push_back(p);
    }
    timer_ = nh_.createTimer( ros::Duration(0.033), &ScanPublisherNode::callbackTimer, this );
    theta_ = 0.0;
    delta_theta_ = 0.02;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "scan_publisher_node");
    ScanPublisherNode sp;
    ros::spin();
    return 0;
}
