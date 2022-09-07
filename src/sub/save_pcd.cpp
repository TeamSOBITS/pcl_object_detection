/* 点群データを受け取る */

/* ROSの基本ヘッダ */
#include <ros/ros.h>                            
/* 入出力関連ヘッダ */
#include <iostream>
/* tf */
#include <tf/transform_listener.h>          //tfリスナーを定義し、座標フレームを処理する
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>            //pcl::PointCloud<T>をROSメッセージとしてPublishおよびSubscribeできる
#include <pcl_ros/transforms.h>             //ポイントクライドを任意の座標フレームに変換する
#include <pcl/point_types.h>                //PCLで実装されたすべてのPointTポイントタイプ構造体を定義する
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
        tf::TransformListener tf_listener_;
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
                        tf_listener_.waitForTransform(target_frame_, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
                        pcl_ros::transformPointCloud(target_frame_, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *cloud_transformed_, tf_listener_);
                    }
                    catch ( const tf::TransformException& ex) {
                        ROS_ERROR("%s", ex.what());
                        return;
                    }
                }
                ROS_INFO("width: %u, height: %u", cloud_transformed_->width, cloud_transformed_->height);
                // 作成したPointCloudをPCD形式で保存する
                std::string path = save_path_+save_file_+"_ascii.pcd";
                ROS_INFO("savePCDFileASCII = '%s'", path.c_str());
                pcl::io::savePCDFileASCII<pcl::PointXYZ> ( path, *cloud_transformed_); // テキスト形式で保存する

                path = save_path_+save_file_+"_binary.pcd";
                ROS_INFO("savePCDFileBinary = '%s'", path.c_str());
                pcl::io::savePCDFileBinary<pcl::PointXYZ> (path, *cloud_transformed_);  // バイナリ形式で保存する
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
