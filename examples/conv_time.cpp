// Open3D
#include <open3d/Open3D.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// open3d_ros
#include "open3d_ros/open3d_ros.h"

// C++
#include <chrono>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open3d_ros_example_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // Example XYZRGB Pointcloud
    sensor_msgs::PointCloud2 ros_pc2;
    open3d::geometry::PointCloud test_o3d_pc;
    open3d::geometry::PointCloud o3d_pc;
    open3d::io::ReadPointCloud("src/open3d_ros/data/fragment.pcd", o3d_pc);

    while (ros::ok())
    {
        // Conversion time from Open3D PointCloud to sensor_msgs::PointCloud2
        auto start_o3d_ros = std::chrono::steady_clock::now();
        open3d_ros::o3d_to_ros(o3d_pc, ros_pc2, "o3d_frame");
        auto end_o3d_ros = std::chrono::steady_clock::now();
        std::cout << "Conversion time (o3d_to_ros) : "
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(end_o3d_ros - start_o3d_ros).count() << " ns"
                  << std::endl;

        // Conversion time from sensor_msgs::PointCloud2 to Open3D PointCloud 
        const sensor_msgs::PointCloud2ConstPtr &ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
        auto start_ros_o3d = std::chrono::steady_clock::now();
        open3d_ros::ros_to_o3d(ros_pc2_ptr, test_o3d_pc);
        auto end_ros_o3d = std::chrono::steady_clock::now();
        std::cout << "Conversion time (ros_to_o3d) : "
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(end_ros_o3d - start_ros_o3d).count()
                  << " ns \n"
                  << std::endl;

        // Visualize result using RViz
        ros_pc2.header.stamp = ros::Time::now();
        pubCloud.publish(ros_pc2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}