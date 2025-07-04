#ifndef POINT_CLOUD_PROCESSOR_HPP
#define POINT_CLOUD_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Core>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pcl_object_detection {
    class PointCloudProcessor {
        protected:
            rclcpp::Node::SharedPtr nd_;

            tf2_ros::Buffer               tfBuffer_;
            tf2_ros::TransformListener    tfListener_;
            tf2_ros::TransformBroadcaster tfBroadcaster_;

            pcl::PassThrough<PointT> pass_;
            pcl::VoxelGrid<PointT> voxel_;
            pcl::search::KdTree<PointT>::Ptr tree_;
            pcl::SACSegmentation<PointT> seg_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::ConcaveHull<PointT> hull_;
            pcl::EuclideanClusterExtraction<PointT> ec_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            laser_geometry::LaserProjection projector_;
            
            std::string base_frame_name_;
            bool publish_cloud_detection_range_;
            bool publish_cloud_object_;
            bool publish_pose_array_;
            bool use_tf_;

            bool use_voxel_;
            double leaf_size_;

            double cluster_tolerance_;
            int min_cluster_point_size_;
            int max_cluster_point_size_;

            double threshold_distance_;
            double probability_;

            double object_size_x_min_;
            double object_size_x_max_;
            double object_size_y_min_;
            double object_size_y_max_;
            double object_size_z_min_;
            double object_size_z_max_;

        public:
            PointCloudProcessor(std::shared_ptr<rclcpp::Node> nd);

            // // Setter functions
            void setPassThroughParameters(const std::string &axis, const float &limit_min, const float &limit_max);
            void setSACPlaneParameter(const std::string &axis, double eps_angle_degree);
            // void setTargetFrame(const std::string& target_frame);
            // void setFlag(const bool need_tf);
            // void setPassThroughParameters(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
            void setVoxelGridParameter();
            void setClusteringParameters();
            void setRadiusOutlierRemovalParameters(double radius, int min_pts, bool keep_organized);
            void setSACSegmentationParameter(int model, int method);
            // void setObjectSizeParameter(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
            // void setObjectOffsetParameter(double x_offset, double y_offset, double z_offset);

            // // Processing functions
            bool transformFramePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud, PointCloud::Ptr output_cloud);
            bool transformFrameScan2D2PointCloud(const sensor_msgs::msg::LaserScan::SharedPtr &input_scan2d, PointCloud::Ptr output_cloud);
            // geometry_msgs::msg::Point transformPoint(const std::string &org_frame, const std::string &target_frame, const geometry_msgs::msg::Point &point);
            bool passThrough(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
            void passThroughXYZ(PointCloud::Ptr cloud, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
            bool voxelGrid(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
            bool euclideanClusterExtraction(const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices);
            bool extractIndices(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative);
            // bool statisticalRemoval(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, int nr_k, double stddev_mult);
            bool radiusOutlierRemoval(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
            bool sacSegmentation(const PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);
            // bool radiusSearch(PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::msg::Point& search_pt, double radius, bool is_accept_add_point);
            bool nearestKSearch(PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::msg::Point& search_pt, int K = 1);
            bool ConcaveHull(const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
            int principalComponentAnalysis(
                const PointCloud::Ptr cloud,
                const std::vector<pcl::PointIndices>& cluster_indices,
                vision_msgs::msg::Detection3DArray::SharedPtr pose_array_msg,
                PointCloud::Ptr cloud_object,
                int init_object_id = 0);
            static bool compareDistance(vision_msgs::msg::Detection3D &a, vision_msgs::msg::Detection3D &b);
            void sendTransform(const geometry_msgs::msg::Pose target_point, const std::string &target_frame);
    };
}


// inline void pcl_object_detection::PointCloudProcessor::setTargetFrame( const std::string& target_frame ) {
//     target_frame_ = target_frame;
// }
// inline void pcl_object_detection::PointCloudProcessor::setFlag( const bool need_tf ) {
//     need_tf_ = need_tf;
// }
// inline void pcl_object_detection::PointCloudProcessor::setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
//     pass_.setFilterFieldName( axis );
//     pass_.setFilterLimits( limit_min, limit_max);
// }
// inline void pcl_object_detection::PointCloudProcessor::setPassThroughParameters( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max ) {
//     pass_param.x_min = x_min;
//     pass_param.x_max = x_max;
//     pass_param.y_min = y_min;
//     pass_param.y_max = y_max;
//     pass_param.z_min = z_min;
//     pass_param.z_max = z_max;
// }
// inline void pcl_object_detection::PointCloudProcessor::setVoxelGridParameter() {
//     voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
// }
// inline void pcl_object_detection::PointCloudProcessor::setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
//     ec_.setClusterTolerance( tolerance );
//     ec_.setMinClusterSize( min_size );
//     ec_.setMaxClusterSize( max_size );
//     ec_.setSearchMethod( tree_ );
// }
// inline void pcl_object_detection::PointCloudProcessor::setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized ) {
//     outrem_.setRadiusSearch( radius );
//     outrem_.setMinNeighborsInRadius ( min_pts );
//     outrem_.setKeepOrganized( keep_organized );
// }
// inline void pcl_object_detection::PointCloudProcessor::setSACSegmentationParameter( const int model,  const int method, const double threshold, const double probability ) {
//     seg_.setOptimizeCoefficients (true);
//     seg_.setModelType (model);
//     seg_.setMethodType (method);
//     seg_.setDistanceThreshold (threshold);
//     seg_.setProbability(probability);
//     seg_.setMaxIterations(1000);
// }
// inline void pcl_object_detection::PointCloudProcessor::setSACPlaneParameter( const std::string &axis, const double eps_angle_degree ) {
//     Eigen::Vector3f axis_vec;
//     if ( axis == "x" ) axis_vec = Eigen::Vector3f(1.0,0.0,0.0); //y axis
//     else if ( axis == "y" ) axis_vec = Eigen::Vector3f(0.0,1.0,0.0); //y axis
//     else if ( axis == "z" ) axis_vec = Eigen::Vector3f(0.0,0.0,1.0); //y axis
//     else return;
//     seg_.setAxis(axis_vec);
//     seg_.setEpsAngle( eps_angle_degree * (M_PI/180.0f) ); // plane can be within eps_angle_degree degrees of plane
// }

// inline void pcl_object_detection::PointCloudProcessor::setObjectSizeParameter( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max ) {
//     obj_param.x_min = x_min;
//     obj_param.x_max = x_max;
//     obj_param.y_min = y_min;
//     obj_param.y_max = y_max;
//     obj_param.z_min = z_min;
//     obj_param.z_max = z_max;
// }
// inline void pcl_object_detection::PointCloudProcessor::setObjectOffsetParameter( const double x_offset, const double y_offset, const double z_offset ) {
//     obj_param.x_offset = x_offset;
//     obj_param.y_offset = y_offset;
//     obj_param.z_offset = z_offset;
// }
// inline bool pcl_object_detection::PointCloudProcessor::compareDistance(sobits_interfaces::msg::ObjectPose &a, sobits_interfaces::msg::ObjectPose &b) {
//     double a_dist = std::hypotf( a.pose.position.x,  a.pose.position.y );
//     double b_dist = std::hypotf( b.pose.position.x,  b.pose.position.y );
//     return a_dist < b_dist; //近い順
// }

#endif // POINT_CLOUD_PROCESSOR_HPP