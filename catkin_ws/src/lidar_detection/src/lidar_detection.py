#!/usr/bin/env python

import rospy
import time
import threading
from sensor_msgs.msg import PointCloud2
from threading import Lock
import os
import sensor_msgs.point_cloud2 as pc2
import math


def callback(data):
    """
    for field in data.fields:
        print(field.name)
        
    test = pc2.read_points(data, skip_nans=True, field_names=("x","y", "z"))
    print(test[0])
"""
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        #print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
        if (math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2] < 3)):
            print(math.tan(p[1]/p[2]))
            print("Proche !!!!!")

def main():


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('LIDAR_detection', anonymous=False)

    rospy.Subscriber("/rs_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()

