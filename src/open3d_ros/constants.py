#!/usr/bin/env python

from sensor_msgs.msg import PointField

fields_xyz = [PointField(name='x', offset=0, datatype=7, count=1),
              PointField(name='y', offset=4, datatype=7, count=1),
              PointField(name='z', offset=8, datatype=7, count=1)]

fields_xyzi = [PointField(name='x', offset=0, datatype=7, count=1),
               PointField(name='y', offset=4, datatype=7, count=1),
               PointField(name='z', offset=8, datatype=7, count=1),
               PointField(name='intensity', offset=12, datatype=4, count=1)]

fields_xyzir = [PointField(name='x', offset=0, datatype=7, count=1),
                PointField(name='y', offset=4, datatype=7, count=1),
                PointField(name='z', offset=8, datatype=7, count=1),
                PointField(name='intensity', offset=16, datatype=7, count=1),
                PointField(name='ring', offset=20, datatype=4, count=1)]
