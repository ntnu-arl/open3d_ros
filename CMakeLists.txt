cmake_minimum_required(VERSION 3.0.2)
project(open3d_ros)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    std_msgs
)

catkin_python_setup()

catkin_package()