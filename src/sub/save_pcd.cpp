#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class SavePCLFileNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    PointCloud::Ptr cloud_transformed_;

    std::string target_frame_;
    std::string save_path_;
    std::string save_file_;

    void cbPoints(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
        try {
            PointCloud cloud_src;
            pcl::fromROSMsg(*pcl_msg, cloud_src);

            if (!target_frame_.empty()) {
                try {
                    geometry_msgs::msg::TransformStamped transform_stamped = 
                        tf_buffer_.lookupTransform(target_frame_, pcl_msg->header.frame_id, tf2::TimePointZero);
                    sensor_msgs::msg::PointCloud2 transformed_msg;
                    tf2::doTransform(*pcl_msg, transformed_msg, transform_stamped);
                    pcl::fromROSMsg(transformed_msg, *cloud_transformed_);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                    return;
                }
            }

            RCLCPP_INFO(this->get_logger(), "width: %u, height: %u", cloud_transformed_->width, cloud_transformed_->height);
            // Save the created PointCloud in PCD format
            std::string path = save_path_ + save_file_ + "_ascii.pcd";
            RCLCPP_INFO(this->get_logger(), "savePCDFileASCII = '%s'", path.c_str());
            pcl::io::savePCDFileASCII<PointT>(path, *cloud_transformed_);

            path = save_path_ + save_file_ + "_binary.pcd";
            RCLCPP_INFO(this->get_logger(), "savePCDFileBinary = '%s'", path.c_str());
            pcl::io::savePCDFileBinary<PointT>(path, *cloud_transformed_);
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

public:
    SavePCLFileNode() : Node("save_pcl_file_node"), tf_listener_(tf_buffer_) {
        this->declare_parameter("pointcloud_topic", "/sensor_data");
        this->declare_parameter("target_frame", "base_footprint");
        this->declare_parameter("save_path", "pcd/");
        this->declare_parameter("save_file", "data");

        this->get_parameter("pointcloud_topic", target_frame_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("save_path", save_path_);
        this->get_parameter("save_file", save_file_);

        RCLCPP_INFO(this->get_logger(), "target_frame = '%s'", target_frame_.c_str());

        sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            target_frame_,
            5,
            std::bind(&SavePCLFileNode::cbPoints, this, std::placeholders::_1)
        );

        cloud_transformed_.reset(new PointCloud());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SavePCLFileNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
