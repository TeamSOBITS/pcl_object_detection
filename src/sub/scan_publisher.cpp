#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ScanPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_sensor_;
    rclcpp::TimerBase::SharedPtr timer_;
    PointCloud::Ptr cloud_;
    double theta_;
    double delta_theta_;

    void callbackTimer();

public:
    ScanPublisherNode();
};

void ScanPublisherNode::callbackTimer() {
    auto cloud_transformed = std::make_shared<PointCloud>();

    theta_ += delta_theta_;
    if (theta_ > 0.6 || theta_ < -0.6) {
        delta_theta_ = -delta_theta_;
    }

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta_);
    Eigen::Quaternionf rotation(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::Vector3f offset(0.0, 0.0, 0.0);

    // Apply rotation and transform
    pcl::transformPointCloud(*cloud_, *cloud_transformed, offset, rotation);

    // Set header and publish
    cloud_transformed->header.frame_id = "base_laser_link";
    pcl_conversions::toPCL(this->now(), cloud_transformed->header.stamp);

    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_transformed, *msg);
    pub_cloud_sensor_->publish(*msg);
}

ScanPublisherNode::ScanPublisherNode() : Node("scan_publisher_node"), theta_(0.0), delta_theta_(0.02) {
    pub_cloud_sensor_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_laserscan", 10);

    cloud_ = std::make_shared<PointCloud>();
    double limit_y = 4.0;
    for (double y = 0.0; y < limit_y; y += 0.01) {
        PointT p;
        p.x = 2.0;
        p.y = y;
        p.z = 0.0;
        cloud_->points.push_back(p);
        p.y = -p.y;
        cloud_->points.push_back(p);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&ScanPublisherNode::callbackTimer, this)
    );
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
