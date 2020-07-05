// Open3D
#include <open3d/Open3D.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// open3d_ros
#include "open3d_ros/open3d_ros.h"

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_data)
{
    open3d::geometry::PointCloud pcd;
    open3d_ros::ros_to_o3d(cloud_data, pcd);
    // Do something with the Open3D pointcloud
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open3d_ros_example_node");
    ros::NodeHandle nh;
    ros::Subscriber subCloud = nh.subscribe("pointcloud2_topic", 1, cloud_callback);
    ros::spin();
}