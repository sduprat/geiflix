#!/usr/bin/env python

import rospy
import numpy as np
import ros_numpy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import math
from std_msgs.msg import Header



pub = rospy.Publisher("coordinates_close", PointCloud2, queue_size=1)




def callback(data):
    global pub


    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    points3D  = np.asarray(points3D.tolist())

    points3D = points3D.reshape(-1,4)

    newPoints = []

    for point in points3D:
        if math.sqrt(point[0]*point[0] +
                                 point[1]*point[1] +
                                 point[2]*point[2]) < 2:
            newPoints.append(point)



    """
    if x > 0: angle = math.degrees(math.atan(y/x))
    elif x < 0: angle = math.degrees(math.atan(y/x) + math.pi)
    else : angle = math.degrees(math.pi/2)
    print(f"Object detected at 20cm at angle {angle}")
    """
            


    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('i', 12, PointField.FLOAT32, 1),
              ]

    header = Header() 
    header.frame_id = "world"
    newData = pc2.create_cloud(header, fields, newPoints)
    newData.header.stamp = data.header.stamp
    pub.publish(newData)

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


