#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <open3d/Open3D.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <open3d/geometry/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

void rospc_to_o3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc)
{
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  if (ros_pc2->fields.size() == 3)
  {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  }
  else if (ros_pc2->fields[3].name == "rgb")
  {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
         ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
    {
      o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      o3d_pc.colors_.push_back(Eigen::Vector3d((float)((int)(*ros_pc2_r)) / 255, (float)((int)(*ros_pc2_g)) / 255,
                                               (float)((int)(*ros_pc2_b)) / 255));
    }
  }
  else
  {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  }
  open3d::visualization::Visualizer visualizer;
  std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud);
  *pointcloud_ptr = o3d_pc;
  visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
  visualizer.AddGeometry(pointcloud_ptr);
  visualizer.Run();
  visualizer.DestroyVisualizerWindow();
}

void o3d_to_rospc(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
                  std::string frame_id)
{
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (pointcloud.HasColors())
  {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  }
  else
  {
    modifier.setPointCloud2FieldsByString(1, "xyz", "rgb");
  }
  modifier.resize(pointcloud.points_.size());
  ros_pc2.header.frame_id = frame_id;
  ros_pc2.height = 1;
  ros_pc2.header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors())
  {
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size();
         i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
    {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      const Eigen::Vector3d& color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = (int)(255 * color(0));
      *ros_pc2_g = (int)(255 * color(1));
      *ros_pc2_b = (int)(255 * color(2));
    }
  }
  else
  {
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_data)
{
  open3d::geometry::PointCloud pcd;
  rospc_to_o3d(cloud_data, pcd);
}

// Uncomment section according to function to test
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_sub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("rospc2_topic", 1);
  /*ros::Subscriber subCloud = nh.subscribe("/l515/depth/color/points", 1, cloud_callback);
  ros::spin();*/

  // Example XYZRGB Pointcloud
  sensor_msgs::PointCloud2 rospc2;
  open3d::geometry::PointCloud pointcloud;
  open3d::io::ReadPointCloud("fragment.pcd", pointcloud);

  while (ros::ok())
  {
    o3d_to_rospc(pointcloud, rospc2, "/o3d_frame");
    pubCloud.publish(rospc2);
  }
}