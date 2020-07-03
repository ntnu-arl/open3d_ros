# open3d_ros

A ROS interface for open3d.

This package is a standalone library that provides functions that can convert pointcloud messages from ROS to open3d and vice-versa.
## Usage

There are two major functions provided in this library:

```cpp
void rospc_to_o3d(ros_pc2,o3d_pc);
void o3d_to_rospc(o3d_pc,ros_pc2,frame_id);
```
## Dependencies:
* Eigen3<br>
* Open3D<br>

## Installation and Troubleshooting:
* Follow the steps mentioned here for building from source http://www.open3d.org/docs/release/compilation.html<br>
 In case your files are not able to link successfully and build fails<br>
Set the flag ```-D_GLIBCXX_USE_CXX11_ABI=0 ``` to 1 in the CMakeLists.txt file in Open3D before starting to build to allow proper linkage in a catkin workspace with ROS
```
cmake -DBUILD_EIGEN3=ON -DBUILD_GLEW=ON -DBUILD_GLFW=ON -DBUILD_JSONCPP=ON -DBUILD_PNG=ON -DGLIBCXX_USE_CXX11_ABI=ON -DPYTHON_EXECUTABLE=/usr/bin/python ..
```
* You may need to upgrade the CMake version.Follow the instrucions below:<br>
1. For Ubuntu Bionic Beaver (18.04):
```
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
sudo apt-get install cmake
```
2. For Ubuntu Xenial Xerus (16.04):
```
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main'
sudo apt-get update
sudo apt-get install cmake
```
### Note: This interface currently only supports XYZ and XYZRGB pointclouds
### Note: On creating a ros pointcloud from an open3d pointcloud it is the user's responsibility to set the timestamp. It is also expected that you would need to pass in the frame id
