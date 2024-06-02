/* Receive point cloud data */

/* ROS Basic Header */
#include <ros/ros.h>
/* I/O Related Headers */
#include <iostream>
/* tf */
#include <tf/transform_listener.h>          //Define a tf listener to process coordinate frames
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>            //Can Publish and Subscribe to pcl::PointCloud<T> as ROS messages
#include <pcl_ros/transforms.h>             //Convert point clyde to arbitrary coordinate frame
#include <pcl/point_types.h>                //Defines all PointT point type structures implemented in PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
/* sensor_msgs */
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class savePCLFileNode {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_points_;
        tf::TransformListener tfListener_;
        PointCloud::Ptr cloud_transformed_;
        std::string target_frame_;
        std::string save_path_;
        std::string save_file_;


        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {
            try {
                PointCloud cloud_src;
                pcl::fromROSMsg(*pcl_msg, cloud_src);

                if (target_frame_.empty() == false) {
                    try {
                        tfListener_.waitForTransform(target_frame_, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
                        pcl_ros::transformPointCloud(target_frame_, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *cloud_transformed_, tfListener_);
                    }
                    catch ( const tf::TransformException& ex) {
                        ROS_ERROR("%s", ex.what());
                        return;
                    }
                }
                ROS_INFO("width: %u, height: %u", cloud_transformed_->width, cloud_transformed_->height);
                // Save the created PointCloud in PCD format
                std::string path = save_path_+save_file_+"_ascii.pcd";
                ROS_INFO("savePCDFileASCII = '%s'", path.c_str());
                pcl::io::savePCDFileASCII<pcl::PointXYZ> ( path, *cloud_transformed_); // Save in text format

                path = save_path_+save_file_+"_binary.pcd";
                ROS_INFO("savePCDFileBinary = '%s'", path.c_str());
                pcl::io::savePCDFileBinary<pcl::PointXYZ> (path, *cloud_transformed_);  // Save in binary format
            } catch (std::exception &e) {
                ROS_ERROR("%s", e.what());
            }
        }

    public:
        savePCLFileNode() : nh_() , pnh_("~") {
            std::string pointcloud_topic = pnh_.param<std::string>( "pointcloud_topic", "/sensor_data" );
            target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
            save_path_ = pnh_.param<std::string>( "save_path", "pcd/" );
            save_file_ = pnh_.param<std::string>( "save_file", "data" );
            ROS_INFO("target_frame = '%s'", target_frame_.c_str());
            ROS_INFO("pointcloud_topic = '%s'", pointcloud_topic.c_str());
            sub_points_ = nh_.subscribe(pointcloud_topic, 5, &savePCLFileNode::cbPoints, this);
            cloud_transformed_.reset(new PointCloud());
        }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "savePCLFile_node");
    savePCLFileNode savePCLFile;
    ros::spin();

}
