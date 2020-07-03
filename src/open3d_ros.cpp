#include "open3d_ros/open3d_ros.h"

namespace open3d_ros
{
    void o3d_to_ros(const open3d::geometry::PointCloud &o3d_pc, sensor_msgs::PointCloud2 &ros_pc2)
    {
        ROSINFO("Converting from o3d to ros");
    }

    void ros_to_o3d(const sensor_msgs::PointCloud2 &ros_pc2, open3d::geometry::PointCloud &o3d_pc)
    {
        ROSINFO("Converting from ros to o3d");
    }

} // namespace open3d_ros
