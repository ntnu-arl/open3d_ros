#ifndef OPEN3D_ROS_HPP_
#define OPEN3D_ROS_HPP_

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <sensor_msgs/PointCloud2.h>

namespace open3d_ros
{
    void o3d_to_ros(const open3d::geometry::PointCloud &o3d_pc, sensor_msgs::PointCloud2 &ros_pc2);

    void ros_to_o3d(const sensor_msgs::PointCloud2 &ros_pc2, open3d::geometry::PointCloud &o3d_pc);
} // namespace open3d_ros

#endif /*OPEN3D_ROS_HPP_*/
