#include <thread>
#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <open3d/Open3D.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <open3d/geometry/PointCloud.h>
#include "open3d_ros/open3d_ros.h"

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_data)
{
    open3d::geometry::PointCloud pcd;
    open3d_ros::ros_to_o3d(cloud_data, pcd);
}

// Uncomment section according to function to test
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_sub");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("rospc2_topic", 1);
    /*ros::Subscriber subCloud = nh.subscribe("pointcloud2_topic", 1, cloud_callback);
    ros::spin();*/

    // Example XYZRGB Pointcloud
    sensor_msgs::PointCloud2 rospc2;
    open3d::geometry::PointCloud pointcloud;
    open3d::io::ReadPointCloud("fragment.pcd", pointcloud);

    while (ros::ok())
    {
        open3d_ros::o3d_to_ros(pointcloud, rospc2, "o3d_frame", ros::Time::now());
        pubCloud.publish(rospc2);
    }
}