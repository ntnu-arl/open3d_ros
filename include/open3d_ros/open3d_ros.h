#ifndef OPEN3D_ROS_HPP_
#define OPEN3D_ROS_HPP_

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Eigen
#include <Eigen/Dense>

// C++
#include <string>

namespace open3d_ros
{
/// \brief Function to create a PointCloud2 from a PointCloud.
/// Transforms a PointCloud to a sensor_msgs::PointCloud2
/// with fields XYZ or XYZRGB depending on the input recieved
/// \param pointcloud The input Open3D PointCloud
/// \param ros_pc2 The ROS PointCloud2 output
/// \param frame_id The frame of reference of the PointCloud2
void o3d_to_ros(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
                std::string frame_id = "open3d_pointcloud");

/// \brief Function to create a PointCloud from a PointCloud2.
/// Transforms a sensor_msgs::PointCloud2 to a PointCloud
/// with fields XYZ or XYZRGB depending on the input recieved
/// \param ros_pc2 The input ROS PointCloud2
/// \param o3d_pc The resultant Open3D PointCloud
/// \param skip_colors If set to true, only XYZ fields will be transferred
void ros_to_o3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                bool skip_colors = false);
}    // namespace open3d_ros

#endif /*OPEN3D_ROS_HPP_*/
