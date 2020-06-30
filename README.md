# open3d_ros

Simply put: A ROS interface for open3d.

This package is a standalone library that provides functions that can convert pointcloud messages from ROS to open3d and vice-versa.

## Usage

There are 2 major functions provided in this library:

```python
ros_to_o3d(ros_pc2)
o3d_to_ros(o3d_pc)
```

## Note: This interface currently only supports XYZI, XYZIR and XYZRGB

## Note: On creating a ros pointcloud from an open3d pointcloud it is the user's responsibility to set the timestamp. It is also expected that you would need to pass in the frame id
