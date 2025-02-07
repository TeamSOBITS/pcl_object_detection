#ifndef BASE_NODE_HPP
#define BASE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sobits_interfaces/msg/object_pose_array.hpp>
#include <sobits_interfaces/srv/mode_ctrl.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include "pcl_object_detection/point_cloud_processor.hpp"

using pcl_object_detection::PointCloudProcessor;

template <typename T1>
class BaseNode : public rclcpp::Node, public std::enable_shared_from_this<BaseNode<T1>>  {
// class BaseNode : public rclcpp::Node  {
public:
    // explicit BaseNode(const std::string &node_name);
    explicit BaseNode(const std::string &node_name, const rclcpp::NodeOptions& options);
    // virtual ~BaseNode() = default;

    virtual void processData(const std::shared_ptr<T1> data) = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

protected:
    void declareCommonParameters();
    void setupCommonPublishers();
    void setupCommonSubscribers();

    std::string pointcloud_topic_;
    std::string laser_topic_;
    std::string target_frame_;
    bool need_cloud_detection_range_;
    bool need_cloud_object_;
    bool need_pose_array_;
    bool use_tf_;

    std::string tabel_param_path_;
    std::string shelf_param_path_;
    std::string floor_param_path_;
    std::string placeable_param_path_;
    std::string line_param_path_;
    std::string common_param_path_;

    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_detection_range_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_object_;
    // rclcpp::Publisher<sobits_interfaces::msg::ObjectPoseArray>::SharedPtr pub_pose_array_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_line_info_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud_detection_range_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud_object_;
    std::shared_ptr<rclcpp::Publisher<sobits_interfaces::msg::ObjectPoseArray>> pub_pose_array_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_marker_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_line_info_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud_line_;

    std::shared_ptr<rclcpp::Subscription<T1>> sub_;

};

template<typename T1>
BaseNode<T1>::BaseNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options) {
    // declareCommonParameters();
}

template<typename T1>
void BaseNode<T1>::declareCommonParameters() {
    

    this->declare_parameter<std::string>("common_param");
    this->declare_parameter<std::string>("table_param");
    this->declare_parameter<std::string>("shelf_param");
    this->declare_parameter<std::string>("floor_param");
    this->declare_parameter<std::string>("placeable_param");
    this->declare_parameter<std::string>("line_param");

    this->get_parameter("common_param", common_param_path_);
    this->get_parameter("table_param", tabel_param_path_);
    this->get_parameter("shelf_param", shelf_param_path_);
    this->get_parameter("floor_param", floor_param_path_);
    this->get_parameter("placeable_param", placeable_param_path_);
    this->get_parameter("line_param", line_param_path_);

    YAML::Node config = YAML::LoadFile(common_param_path_);

    pointcloud_topic_ = config["pointcloud_topic_name"].as<std::string>();
    laser_topic_ = config["scan_topic_name"].as<std::string>();
    target_frame_ = config["base_frame_name"].as<std::string>();
    need_cloud_detection_range_ = config["publish_cloud_detection_range"].as<bool>();
    need_cloud_object_ = config["publish_cloud_object"].as<bool>();
    need_pose_array_ = config["publish_pose_array"].as<bool>();
    use_tf_ = config["use_tf"].as<bool>();

}

template<typename T1>
void BaseNode<T1>::setupCommonPublishers() {
    pub_cloud_detection_range_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_detection_range", 10);
    pub_cloud_object_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_object", 10);
    pub_pose_array_ = this->create_publisher<sobits_interfaces::msg::ObjectPoseArray>("object_poses", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("line_info_marker", 10);
    pub_line_info_ = this->create_publisher<std_msgs::msg::Float64>("line_info", 10);
    pub_cloud_line_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_line_topic", 10);

}

template <typename T1>
void BaseNode<T1>::setupCommonSubscribers() {
    if constexpr (std::is_same<T1, sensor_msgs::msg::PointCloud2>::value) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to PointCloud2 topic: %s", pointcloud_topic_.c_str());
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->processData(msg);
            });
    } else if constexpr (std::is_same<T1, sensor_msgs::msg::LaserScan>::value) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to LaserScan topic: %s", laser_topic_.c_str());
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->processData(msg);
            });
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported message type for subscription.");
    }
}

#endif // BASE_NODE_HPP