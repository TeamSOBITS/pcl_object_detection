#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

enum Mode {
    OFF = 0,
    TABLE_MODE,
    FLOOR_MODE,
    SHELF_MODE,
    PLACEABLE_POSITION
};

class PointcloudPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_sensor_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::TimerBase::SharedPtr timer_;

    PointCloud::Ptr cloud_;
    pcl::VoxelGrid<PointT> voxel_;

    std::string table_pcd_path_;
    std::string floor_pcd_path_;
    std::string shelf_pcd_path_;
    std::string placeable_pcd_path_;

    int detection_mode_;

    void loadPointCloud(const std::string &path, PointCloud::Ptr cloud);
    void timerCallback();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

public:
    PointcloudPublisherNode();
};

void PointcloudPublisherNode::loadPointCloud(const std::string &path, PointCloud::Ptr cloud) {
    RCLCPP_INFO(this->get_logger(), "Loading PointCloud from: %s", path.c_str());
    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", path.c_str());
    }
}

void PointcloudPublisherNode::timerCallback() {
    auto sensor_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto cloud_downsampling = std::make_shared<PointCloud>();

    voxel_.setInputCloud(cloud_);
    voxel_.filter(*cloud_downsampling);

    cloud_downsampling->header.frame_id = "camera_link";
    pcl::toROSMsg(*cloud_downsampling, *sensor_cloud);

    auto now = this->now();
    sensor_cloud->header.stamp = now;

    pub_cloud_->publish(*sensor_cloud);
    pub_cloud_sensor_->publish(*sensor_cloud);
}

rcl_interfaces::msg::SetParametersResult PointcloudPublisherNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &param : parameters) {
        if (param.get_name() == "detection_mode") {
            detection_mode_ = param.as_int();
            if (detection_mode_ == Mode::TABLE_MODE) {
                loadPointCloud(table_pcd_path_, cloud_);
            } else if (detection_mode_ == Mode::FLOOR_MODE) {
                loadPointCloud(floor_pcd_path_, cloud_);
            } else if (detection_mode_ == Mode::SHELF_MODE) {
                loadPointCloud(shelf_pcd_path_, cloud_);
            } else if (detection_mode_ == Mode::PLACEABLE_POSITION) {
                loadPointCloud(placeable_pcd_path_, cloud_);
            }
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

PointcloudPublisherNode::PointcloudPublisherNode() : Node("pointcloud_publisher_node") {
    pub_cloud_sensor_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_downsampling", 10);

    cloud_.reset(new PointCloud());
    voxel_.setLeafSize(0.01f, 0.01f, 0.01f);

    this->declare_parameter("detection_mode", Mode::OFF);
    this->declare_parameter("table_pcd_path", "table.pcd");
    this->declare_parameter("floor_pcd_path", "floor.pcd");
    this->declare_parameter("shelf_pcd_path", "shelf.pcd");
    this->declare_parameter("placeable_pcd_path", "placeable.pcd");

    detection_mode_ = this->get_parameter("detection_mode").as_int();
    table_pcd_path_ = this->get_parameter("table_pcd_path").as_string();
    floor_pcd_path_ = this->get_parameter("floor_pcd_path").as_string();
    shelf_pcd_path_ = this->get_parameter("shelf_pcd_path").as_string();
    placeable_pcd_path_ = this->get_parameter("placeable_pcd_path").as_string();

    this->add_on_set_parameters_callback(std::bind(&PointcloudPublisherNode::parametersCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&PointcloudPublisherNode::timerCallback, this));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointcloudPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}