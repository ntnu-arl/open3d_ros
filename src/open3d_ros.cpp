#include "open3d_ros/open3d_ros.h"
#include <ros/ros.h>
namespace open3d_ros
{
void o3d_to_ros(const open3d::geometry::PointCloud& o3d_pc, sensor_msgs::PointCloud2& ros_pc2)
{
  ROS_INFO("Converting from o3d to ros");
}

void ros_to_o3d(const sensor_msgs::PointCloud2& ros_pc2, open3d::geometry::PointCloud& o3d_pc)
{
  ROS_INFO("Converting from ros to o3d");
}

}  // namespace open3d_ros
