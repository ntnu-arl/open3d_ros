#/usr/bin/env python

import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from constants import *


def ros_to_o3d(ros_pc2):
    points = list(read_points(ros_pc2))
    field_names = [field.name for field in ros_pc2.fields]
    o3d_pc = o3d.geometry.PointCloud()
    if len(points) == 0:
        # Read an empty cloud
        return o3d_pc

    if 'intensity' in field_names:
        if 'ring' in field_names:
            xyz = [(x, y, z) for x, y, z, _, _ in points]
        else:
            xyz = [(x, y, z) for x, y, z, _ in points]

    o3d_pc.points = o3d.utility.Vector3dVector(np.array(xyz))
    return o3d_pc


def o3d_to_ros(o3d_pc, frame_id='open3d_pointcloud'):
    header = Header()
    header.frame_id = frame_id
    points = np.asarray(o3d_pc.points)
    return create_cloud(header, fields_xyz, points)
