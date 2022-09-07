#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_object_detection/ObjectDetectionParameterConfig.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

enum Mode {
    OFF = 0,
    TABLE_MODE,
    FLOOR_MODE,
    SHELF_MODE,
    PLACEABLE_POSITION
};

class PointcloudPublisherNode
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Timer timer_;
        ros::Publisher pub_cloud_sensor_;
        ros::Publisher pub_cloud_;
        PointCloud::Ptr cloud_;
        pcl::VoxelGrid<PointT> voxel_;

        dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>* server_;
        dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>::CallbackType f_;

        void callbackDynamicReconfigure(pcl_object_detection::ObjectDetectionParameterConfig& config, uint32_t level);
        void loadPointCloud( std::string path, PointCloud::Ptr cloud );
        void callbackTimer(const ros::TimerEvent& e);
    public:
        PointcloudPublisherNode();
};

void PointcloudPublisherNode::callbackDynamicReconfigure(pcl_object_detection::ObjectDetectionParameterConfig& config, uint32_t level) {
    if( config.detection_mode == Mode::TABLE_MODE ) loadPointCloud( pnh_.param<std::string>( "table_pcd_path", "table.pcd" ), cloud_ );
    else if( config.detection_mode == Mode::FLOOR_MODE ) loadPointCloud( pnh_.param<std::string>( "floor_pcd_path", "floor.pcd" ), cloud_ );
    else if( config.detection_mode == Mode::SHELF_MODE ) loadPointCloud( pnh_.param<std::string>( "shelf_pcd_path", "shelf.pcd" ), cloud_ );
    else if ( config.detection_mode == Mode::PLACEABLE_POSITION ) loadPointCloud( pnh_.param<std::string>( "placeable_pcd_path", "placeable.pcd" ), cloud_ );
    return;
}

void PointcloudPublisherNode::loadPointCloud( std::string path, PointCloud::Ptr cloud ) {
    std::cout << "====================\nLoad Data" << std::endl;
    std::cout << "data_path = "  << path << std::endl;
    std::cout << "====================\n" << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    return;
}

void PointcloudPublisherNode::callbackTimer(const ros::TimerEvent& e){
    sensor_msgs::PointCloud2Ptr sensor_cloud( new  sensor_msgs::PointCloud2 );
    PointCloud::Ptr cloud_downsampling( new PointCloud() );
    pcl::toROSMsg(*cloud_, *sensor_cloud);

    voxel_.setInputCloud( cloud_ );
    voxel_.filter( *cloud_downsampling );
    cloud_downsampling->header.frame_id = "camera_link";
    sensor_cloud->header.frame_id = "camera_link";

    pcl_conversions::toPCL(ros::Time::now(), cloud_downsampling->header.stamp);
    sensor_cloud->header.stamp = ros::Time::now();
    pub_cloud_.publish(cloud_downsampling);
    pub_cloud_sensor_.publish(sensor_cloud);
    return;
}

PointcloudPublisherNode::PointcloudPublisherNode() : nh_(), pnh_("~") {
    pub_cloud_sensor_ = nh_.advertise<sensor_msgs::PointCloud2>("/points2", 1);
    pub_cloud_ = nh_.advertise<PointCloud>("/cloud_downsampling", 1);
    cloud_.reset( new PointCloud() );
    voxel_.setLeafSize( 0.01, 0.01, 0.01 );
    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<pcl_object_detection::ObjectDetectionParameterConfig>(pnh_);
    f_ = boost::bind(&PointcloudPublisherNode::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    timer_ = nh_.createTimer( ros::Duration(0.033), &PointcloudPublisherNode::callbackTimer, this );
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pointcloud_publisher_node");
    PointcloudPublisherNode pp;
    ros::spin();
    return 0;
}
