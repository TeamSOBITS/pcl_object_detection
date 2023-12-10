#ifndef POINT_CLOUD_PROCESSOR_HPP
#define POINT_CLOUD_PROCESSOR_HPP

#include <tf2_ros/transform_listener.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <pcl_object_detection/RunCtrl.h>
#include <pcl_object_detection/ObjectPose.h>
#include <pcl_object_detection/ObjectPoseArray.h>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct PassthroughParameter {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
};

struct ObjectSizeParameter {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
    double x_offset;
    double y_offset;
    double z_offset;
};

namespace pcl_object_detection {
    class PointCloudProcessor {
        protected:
            tf2_ros::Buffer tfBuffer_;
            tf2_ros::TransformListener tfListener_;
            laser_geometry::LaserProjection projector_;
            pcl::PassThrough<PointT> pass_;
            pcl::VoxelGrid<PointT> voxel_;
            pcl::search::KdTree<PointT>::Ptr tree_;
            pcl::EuclideanClusterExtraction<PointT> ec_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::SACSegmentation<PointT> seg_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::ConcaveHull<PointT> hull_;

            tf2_ros::TransformBroadcaster broadcaster_;
            std::string target_frame_;
            bool need_tf_;

        public:
            PassthroughParameter pass_param;
            ObjectSizeParameter obj_param;

            PointCloudProcessor();
            void setTargetFrame( const std::string& target_frame );
            void setFlag( const bool need_tf );
            void setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max );
            void setPassThroughParameters( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max );
            void setVoxelGridParameter( const float leaf_size );
            void setClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            void setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized );
            void setSACSegmentationParameter( const int model,  const int method, const double threshold, const double probability );
            void setSACPlaneParameter( const std::string &axis, const double eps_angle_degree );
            void setObjectSizeParameter( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max );
            void setObjectOffsetParameter( const double x_offset, const double y_offset, const double z_offset );

            bool transformFramePointCloud ( const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud );
            bool transformFrameScan2D2PointCloud ( const sensor_msgs::LaserScanConstPtr &input_scan2d, PointCloud::Ptr output_cloud );
            geometry_msgs::Point transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point );
            bool passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            void passThroughXYZ( PointCloud::Ptr cloud );
            bool voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            bool euclideanClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
            bool extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative );
            bool statisticalRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const int nr_k, const double stddev_mult );
            bool radiusOutlierRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            bool sacSegmentation( const PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients );
            bool radiusSearch ( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::Point& search_pt, const double radius, bool is_accept_add_point );
            bool nearestKSearch( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, const geometry_msgs::Point& search_pt, const int K = 1 );
            bool ConcaveHull( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
            int principalComponentAnalysis(
                const PointCloud::Ptr cloud,
                const std::vector<pcl::PointIndices>& cluster_indices,
                pcl_object_detection::ObjectPoseArrayPtr pose_array_msg,
                PointCloud::Ptr cloud_object,
                const int init_object_id = 0 );
            static bool compareDistance(pcl_object_detection::ObjectPose &a, pcl_object_detection::ObjectPose &b);
    };

    inline void PointCloudProcessor::setTargetFrame( const std::string& target_frame ) {
        target_frame_ = target_frame;
    }
    inline void PointCloudProcessor::setFlag( const bool need_tf ) {
        need_tf_ = need_tf;
    }
    inline void PointCloudProcessor::setPassThroughParameters( const std::string &axis, const float &limit_min, const float &limit_max ) {
        pass_.setFilterFieldName( axis );
        pass_.setFilterLimits( limit_min, limit_max);
    }
    inline void PointCloudProcessor::setPassThroughParameters( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max ) {
        pass_param.x_min = x_min;
        pass_param.x_max = x_max;
        pass_param.y_min = y_min;
        pass_param.y_max = y_max;
        pass_param.z_min = z_min;
        pass_param.z_max = z_max;
    }
    inline void PointCloudProcessor::setVoxelGridParameter( const float leaf_size ) {
        voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
    }
    inline void PointCloudProcessor::setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_.setClusterTolerance( tolerance );
        ec_.setMinClusterSize( min_size );
        ec_.setMaxClusterSize( max_size );
        ec_.setSearchMethod( tree_ );
    }
    inline void PointCloudProcessor::setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized ) {
        outrem_.setRadiusSearch( radius );
        outrem_.setMinNeighborsInRadius ( min_pts );
        outrem_.setKeepOrganized( keep_organized );
    }
    inline void PointCloudProcessor::setSACSegmentationParameter( const int model,  const int method, const double threshold, const double probability ) {
        seg_.setOptimizeCoefficients (true);
        seg_.setModelType (model);
        seg_.setMethodType (method);
        seg_.setDistanceThreshold (threshold);
        seg_.setProbability(probability);
        seg_.setMaxIterations(1000);
    }
    inline void PointCloudProcessor::setSACPlaneParameter( const std::string &axis, const double eps_angle_degree ) {
        Eigen::Vector3f axis_vec;
        if ( axis == "x" ) axis_vec = Eigen::Vector3f(1.0,0.0,0.0); //y axis
        else if ( axis == "y" ) axis_vec = Eigen::Vector3f(0.0,1.0,0.0); //y axis
        else if ( axis == "z" ) axis_vec = Eigen::Vector3f(0.0,0.0,1.0); //y axis
        else return;
        seg_.setAxis(axis_vec);
        seg_.setEpsAngle( eps_angle_degree * (M_PI/180.0f) ); // plane can be within eps_angle_degree degrees of plane
    }

    inline void PointCloudProcessor::setObjectSizeParameter( const double x_min, const double x_max, const double y_min, const double y_max, const double z_min, const double z_max ) {
        obj_param.x_min = x_min;
        obj_param.x_max = x_max;
        obj_param.y_min = y_min;
        obj_param.y_max = y_max;
        obj_param.z_min = z_min;
        obj_param.z_max = z_max;
    }
    inline void PointCloudProcessor::setObjectOffsetParameter( const double x_offset, const double y_offset, const double z_offset ) {
        obj_param.x_offset = x_offset;
        obj_param.y_offset = y_offset;
        obj_param.z_offset = z_offset;
    }
    inline bool PointCloudProcessor::compareDistance(pcl_object_detection::ObjectPose &a, pcl_object_detection::ObjectPose &b) {
        double a_dist = std::hypotf( a.pose.position.x,  a.pose.position.y );
        double b_dist = std::hypotf( b.pose.position.x,  b.pose.position.y );
        return a_dist < b_dist; //近い順
    }

}

#endif