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

    // Filter points to 2D for PCA in XY plane
    Eigen::MatrixXf points_2d(2, cluster.indices.size());
    float min_z = std::numeric_limits<float>::max(), max_z = -min_z;

    for (size_t i = 0; i < cluster.indices.size(); ++i) {
      const auto& pt = cloud->points[cluster.indices[i]];
      points_2d(0, i) = pt.x - centroid[0];
      points_2d(1, i) = pt.y - centroid[1];
      min_z = std::min(min_z, pt.z);
      max_z = std::max(max_z, pt.z);
    }

    // Compute 2x2 Covariance Matrix
    Eigen::Matrix2f covariance = (points_2d * points_2d.transpose()) / static_cast<float>(cluster.indices.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    Eigen::Matrix2f eigenvectors = solver.eigenvectors();

    // The eigenvector with the largest eigenvalue is the major axis
    // solver.eigenvectors() is sorted by eigenvalues ascending, so col(1) is major
    Eigen::Vector2f major_axis = eigenvectors.col(1);
    Eigen::Vector2f minor_axis = eigenvectors.col(0);

    // Compute horizontal dimensions by projecting onto these axes
    float min_major = std::numeric_limits<float>::max(), max_major = -min_major;
    float min_minor = min_major, max_minor = max_major;

    for (size_t i = 0; i < cluster.indices.size(); ++i) {
      float proj_major = major_axis.dot(points_2d.col(i));
      float proj_minor = minor_axis.dot(points_2d.col(i));
      min_major = std::min(min_major, proj_major);
      max_major = std::max(max_major, proj_major);
      min_minor = std::min(min_minor, proj_minor);
      max_minor = std::max(max_minor, proj_minor);
    }

    double yaw = std::atan2(major_axis.y(), major_axis.x());

    // Populate Output Box
    box.center.position.x = centroid[0];
    box.center.position.y = centroid[1];
    box.center.position.z = (max_z + min_z) / 2.0; // Vertical center
    
    box.center.orientation.x = 0.0;
    box.center.orientation.y = 0.0;
    box.center.orientation.z = std::sin(yaw / 2.0);
    box.center.orientation.w = std::cos(yaw / 2.0);
    
    // Here x is the Major axis, y is the Minor axis
    box.size.x = max_major - min_major;
    box.size.y = max_minor - min_minor;
    box.size.z = max_z - min_z;

    return box;
  }
};

}  // namespace pcl_object_detection
