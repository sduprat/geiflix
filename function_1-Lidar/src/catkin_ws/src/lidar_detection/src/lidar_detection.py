#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math

def callback(data):

    for coordinates in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        x = coordinates[0]
        y = coordinates[1]
        z = coordinates[2]

        if (math.sqrt(x*x + y*y + z*z < 0.2)):
            if x > 0: angle = math.degrees(math.atan(y/x))
            elif x < 0: angle = math.degrees(math.atan(y/x) + math.pi)
            else : angle = math.degrees(math.pi/2)
            print(f"Object detected at 20cm at angle {angle}")


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('LIDAR_detection', anonymous=False)
    rospy.Subscriber("/rslidar_points", PointCloud2, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()


