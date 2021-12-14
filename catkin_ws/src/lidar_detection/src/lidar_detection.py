#!/usr/bin/env python

import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import math
from std_msgs.msg import Header



pub = rospy.Publisher("coordinates_close", PointCloud2, queue_size=10)




def callback(data):
    global pub
    for coordinates in pc2.read_points(data, field_names = ("x", "y", "z","intensity"), skip_nans=True):
        x = coordinates[0]
        y = coordinates[1]
        z = coordinates[2]
        I = coordinates[3]

        if (math.sqrt(x*x + y*y + z*z < 1)):
            """
            if x > 0: angle = math.degrees(math.atan(y/x))
            elif x < 0: angle = math.degrees(math.atan(y/x) + math.pi)
            else : angle = math.degrees(math.pi/2)
            print(f"Object detected at 20cm at angle {angle}")
            """
            
            points = [coordinates]


            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('i', 12, PointField.FLOAT32, 1),
                      ]

            header = Header() 
            header.frame_id = "map"
            data = pc2.create_cloud(header, fields, points)
            data.header.stamp = rospy.Time.now()
            pub.publish(data)

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


