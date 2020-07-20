// Open3D
#include <open3d/Open3D.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

// open3d_ros
#include "open3d_ros/open3d_ros.h"

// C++
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open3d_ros_example_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    while (ros::ok())
    {
        // Example XYZRGB Pointcloud
        sensor_msgs::PointCloud2 ros_pc2;
        open3d::geometry::PointCloud test_o3d_pc;
        open3d::geometry::PointCloud o3d_pc;
        std::string path = ros::package::getPath("open3d_ros");
        open3d::io::ReadPointCloud(path + "/data/fragment.pcd", o3d_pc);
        // Conversion time from Open3D PointCloud to sensor_msgs::PointCloud2
        std::clock_t tic = std::clock();
        open3d_ros::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
        std::cout << "Conversion time (open3dToRos) : " << float(std::clock() - tic) / CLOCKS_PER_SEC * 1000.0 << " ms"
                  << std::endl;

        // Conversion time from sensor_msgs::PointCloud2 to Open3D PointCloud
        const sensor_msgs::PointCloud2ConstPtr &ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
        std::clock_t toc = std::clock();
        open3d_ros::rosToOpen3d(ros_pc2_ptr, test_o3d_pc);
        std::cout << "Conversion time (rosToOpen3d) : " << float(std::clock() - toc) / CLOCKS_PER_SEC * 1000.0
                  << " ms \n"
                  << std::endl;

        std::cout << "Number of points: " << o3d_pc.points_.size() << std::endl;

        // Visualize result using RViz
        ros_pc2.header.stamp = ros::Time::now();
        pubCloud.publish(ros_pc2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}