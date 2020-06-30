#!/usr/bin/env python

import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from constants import *
from ctypes import c_uint32
import struct


def float_to_rgb(f):
    s = struct.pack('>f', f)
    i = struct.unpack('>l', s)[0]
    pack = c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    color = [r, g, b]
    return color


def ros_to_o3d(ros_pc2):
    points = np.array(list(read_points(ros_pc2)))
    field_names = [field.name for field in ros_pc2.fields]
    o3d_pc = o3d.geometry.PointCloud()
    if len(points) == 0:
        # Empty cloud
        return o3d_pc
    if 'rgb' in field_names:
        xyz = points[:, :3]
        rgb = [float_to_rgb(f) for f in points[:, 3]]
        o3d_pc.points = o3d.utility.Vector3dVector(np.array(xyz))
        o3d_pc.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
        return o3d_pc
    elif 'intensity' in field_names:
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
