// Open3D
#include <open3d/Open3D.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

// open3d_ros
#include "open3d_ros/open3d_ros.h"

// C++
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open3d_ros_example_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // Example XYZRGB Pointcloud
    sensor_msgs::PointCloud2 ros_pc2;
    open3d::geometry::PointCloud o3d_pc;
    std::string path = ros::package::getPath("open3d_ros");
    open3d::io::ReadPointCloud(path + "/data/fragment.pcd", o3d_pc);
    open3d_ros::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
    while (ros::ok())
    {
        ros_pc2.header.stamp = ros::Time::now();
        pubCloud.publish(ros_pc2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}