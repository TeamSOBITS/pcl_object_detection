#pragma once

#include <string>
#include <algorithm>
#include <limits>
#include <cmath>

// PCL Core and Math
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

// PCL Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// Eigen for PCA Math
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS 2 Messages
#include <vision_msgs/msg/bounding_box3_d.hpp>

namespace pcl_object_detection {

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

/**
 * @class PointCloudUtility
 * @brief Stateless, high-performance utility functions for Point Cloud Processing.
 */
class PointCloudUtility {
public:

  /**
   * @brief Applies a VoxelGrid downsampling filter safely.
   * @param in Input point cloud.
   * @param out Output point cloud.
   * @param leaf The voxel size (e.g., 0.02 for 2cm). If <= 0, no filtering is applied.
   */
  static void applyVoxelGrid(const PointCloud::Ptr& in, PointCloud::Ptr& out, float leaf) {
    if (leaf <= 0.0f) {
      if (in != out) *out = *in;
      return;
    }
    pcl::VoxelGrid<PointT> v;
    v.setInputCloud(in);
    v.setLeafSize(leaf, leaf, leaf);
    v.filter(*out);
  }

  /**
   * @brief Applies a PassThrough filter to crop a point cloud along a specific axis.
   * @param in Input point cloud.
   * @param out Output point cloud.
   * @param axis The axis to crop ("x", "y", or "z").
   * @param min Minimum allowable value.
   * @param max Maximum allowable value.
   */
  static void applyPassThrough(
    const PointCloud::Ptr& in, 
    PointCloud::Ptr& out, 
    const std::string& axis, 
    float min, 
    float max) 
  {
    pcl::PassThrough<PointT> p;
    p.setInputCloud(in);
    p.setFilterFieldName(axis);
    p.setFilterLimits(min, max);
    p.filter(*out);
  }

  /**
   * @brief Computes a highly-optimized, 3D Oriented Bounding Box using PCA.
   * 
   * This uses Eigen math to find the principal axes of the cluster, and then 
   * manually projects the points onto these axes to find the tightest 
   * bounding box dimensions without the overhead of copying/transforming the cloud.
   * 
   * @param cloud The full point cloud containing the objects.
   * @param cluster The specific indices belonging to the single object.
   * @return A standard vision_msgs BoundingBox3D populated with Size, Center, and Orientation.
   */
  static vision_msgs::msg::BoundingBox3D computePCAAlignedBox(
    const PointCloud::Ptr& cloud, 
    const pcl::PointIndices& cluster) 
  {
    vision_msgs::msg::BoundingBox3D box;
    
    // Find the Center of Mass (Centroid)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, cluster, centroid);

    // Compute the Covariance Matrix and Extract Eigenvectors (Principal Axes)
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, cluster, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f vectors = solver.eigenvectors();

    // Manual Projection for Bounding Box Limits:
    // Instead of creating a whole new transformed cloud (which is slow), we project 
    // each point onto the local Eigen vector frame to find the max/min dimensions.
    float min_x = std::numeric_limits<float>::max(), max_x = -min_x;
    float min_y = min_x, max_y = max_x;
    float min_z = min_x, max_z = max_x;

    for (const auto& idx : cluster.indices) {
      Eigen::Vector3f pt = cloud->points[idx].getVector3fMap();
      
      // Project the point into the local PCA coordinate system
      Eigen::Vector3f local_pt = vectors.transpose() * (pt - centroid.head<3>());
      
      min_x = std::min(min_x, local_pt.x()); 
      max_x = std::max(max_x, local_pt.x());
      min_y = std::min(min_y, local_pt.y()); 
      max_y = std::max(max_y, local_pt.y());
      min_z = std::min(min_z, local_pt.z()); 
      max_z = std::max(max_z, local_pt.z());
    }

    // Extract Orientation (Yaw):
    // Eigenvectors are sorted by eigenvalue, so column 0 is the minor axis, 
    // and column 2 is the major axis. We extract the Yaw angle from the primary plane.
    double yaw = std::atan2(vectors(1, 0), vectors(0, 0));

    // Populate Output Box
    box.center.position.x = centroid[0];
    box.center.position.y = centroid[1];
    box.center.position.z = centroid[2];
    
    // Convert Yaw to Quaternion (Roll=0, Pitch=0)
    box.center.orientation.x = 0.0;
    box.center.orientation.y = 0.0;
    box.center.orientation.z = std::sin(yaw / 2.0);
    box.center.orientation.w = std::cos(yaw / 2.0);
    
    box.size.x = max_x - min_x;
    box.size.y = max_y - min_y;
    box.size.z = max_z - min_z;

    return box;
  }
};

}  // namespace pcl_object_detection
