#include <gtest/gtest.h>

#include "open3d_ros/open3d_ros.h"
#include <open3d/Open3D.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>

TEST(Open3D_ROS, o3d_to_ros_to_o3d)
{
    open3d::geometry::PointCloud o3d_pcd;
    open3d::geometry::PointCloud ros_o3d_pcd;
    sensor_msgs::PointCloud2 rospc2;
    open3d::io::ReadPointCloud("src/open3d_ros/data/fragment.pcd", o3d_pcd);
    open3d_ros::o3d_to_ros(o3d_pcd, rospc2, "o3d_frame");
    EXPECT_EQ(rospc2.height*rospc2.width, o3d_pcd.points_.size());
    const sensor_msgs::PointCloud2ConstPtr &rospc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(rospc2);
    EXPECT_EQ(rospc2_ptr->height*rospc2_ptr->width, o3d_pcd.points_.size());
    open3d_ros::ros_to_o3d(rospc2_ptr, ros_o3d_pcd);
    for (unsigned int i = 0; i < o3d_pcd.points_.size(); i++)
    {
        const Eigen::Vector3d &point = o3d_pcd.points_[i];
        const Eigen::Vector3d &ros_point = ros_o3d_pcd.points_[i];
        EXPECT_EQ(point(0), ros_point(0));
        EXPECT_EQ(point(1), ros_point(1));
        EXPECT_EQ(point(2), ros_point(2));
        if (o3d_pcd.HasColors())
        {
            const Eigen::Vector3d &color = o3d_pcd.colors_[i];
            const Eigen::Vector3d &ros_color = ros_o3d_pcd.colors_[i];
            EXPECT_EQ(color(0), ros_color(0));
            EXPECT_EQ(color(1), ros_color(1));
            EXPECT_EQ(color(2), ros_color(2));
        }
    }
}

TEST(Open3D_ROS_2, o3d_to_ros_to_o3d)
{
    open3d::geometry::PointCloud o3d_pcd;
    open3d::geometry::PointCloud ros_o3d_pcd;
    sensor_msgs::PointCloud2 rospc2;
    o3d_pcd.points_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(1.0, 1.0, 0.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));
    o3d_pcd.points_.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

    open3d_ros::o3d_to_ros(o3d_pcd, rospc2, "o3d_frame");
    EXPECT_EQ(rospc2.height*rospc2.width, o3d_pcd.points_.size());
    const sensor_msgs::PointCloud2ConstPtr &rospc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(rospc2);
    EXPECT_EQ(rospc2_ptr->height*rospc2_ptr->width, o3d_pcd.points_.size());
    open3d_ros::ros_to_o3d(rospc2_ptr, ros_o3d_pcd);
    for (unsigned int i = 0; i < o3d_pcd.points_.size(); i++)
    {
        const Eigen::Vector3d &point = o3d_pcd.points_[i];
        const Eigen::Vector3d &ros_point = ros_o3d_pcd.points_[i];
        EXPECT_EQ(point(0), ros_point(0));
        EXPECT_EQ(point(1), ros_point(1));
        EXPECT_EQ(point(2), ros_point(2));
        if (o3d_pcd.HasColors())
        {
            const Eigen::Vector3d &color = o3d_pcd.colors_[i];
            const Eigen::Vector3d &ros_color = ros_o3d_pcd.colors_[i];
            EXPECT_EQ(color(0), ros_color(0));
            EXPECT_EQ(color(1), ros_color(1));
            EXPECT_EQ(color(2), ros_color(2));
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}