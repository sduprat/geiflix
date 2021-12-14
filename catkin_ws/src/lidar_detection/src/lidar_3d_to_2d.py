#!/usr/bin/env python


import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs import PointCloud2, PointField
import numpy as np
import ros_numpy
import math
from std_msgs import Header


pub = rospy.Publisher("coordinates_lidar_2d", PointCloud2, queue_size=10)


def callback(data):


    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    points3D = np.asarray(points3D.tolist())

    #FIlter points
    inrange = np.where((points3D[:, 2] > -1) &
                       (points3D[:, 2] < 3) &
                       (np.abs(points3D[:, 0]) < 3) &
                       (np.abs(points3D[:, 1]) < 3))
    max_intensity = np.max(points3D[:, 3])
    points3D = points3D[inrange[0]]


    # Color map for the points
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points3D[:, 3] / max_intensity) * 255


    # Project to 2D and filter points within image boundaries
    points2D = [ CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3], math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2] for point in points3D[:, :3]]
    points2D = np.asarray(points2D)
    inrange = np.where((points2D[:, 0] >= 0) &
                       (points2D[:, 1] >= 0) &
                       (points2D[:, 0] < img.shape[1]) &
                       (points2D[:, 1] < img.shape[0]))
    points2D = points2D[inrange[0]].round().astype('int')

    fields = [PointField('x', 0, PointField.INT32, 1),
              PointField('y', 4, PointField.INT32, 1),
              PointField('distance', 8, PointField.FLOAT32, 1),
              ]


    header = Header()
    header.frame_id = "map"
    data = pc2.create_cloud(header, fields, points)
    data.header.stamp = rospy.Time.now()
    pub.publisher(data)



def main():
    
    rospy.init_node('LIDAR_3D_2D', anonymous=False)
    rospy.Subscriber("/rslidar_points_transfo", PointCloud2, callback)


    try:
        rospy.spin()
    except rospy.ROSInterruptException
     rospy.loginfo('Shutting down')

if __name__ == '__main__':
    main()



















