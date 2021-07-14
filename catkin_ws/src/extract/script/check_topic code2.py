#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import pcl
from rospy.numpy_msg import numpy_msg
import cv2
import numpy as np
from projection_helper import birds_eye_point_cloud
import copy
import pyproj as pj
from numpy.polynomial import polynomial as P
from scipy import stats
from sklearn.cluster import KMeans, DBSCAN
import matplotlib
# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
#$DISPLAY error, solution:- echo "backend: Agg" > ~/.config/matplotlib/matplotlibrc
from matplotlib import pyplot as plt
from matplotlib import colors as clr
from mpl_toolkits.mplot3d import Axes3D

import tf2_ros

class CheckTopic:
    counter = 0
    def __init__(self):
        rospy.init_node("Starting the CheckTopic code")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        #rospy.Subscriber('/ibeo/lidar/dynamic', numpy_msg(PointCloud2), self.check_topic) 
        #rospy.Subscriber('/lidar_pointcloud/top', numpy_msg(PointCloud2), self.check_topic) 
        rospy.Subscriber('/pointcloud_transformer/output_pcl2', numpy_msg(PointCloud2), self.check_topic)
        rospy.spin()
        
    def check_topic(self, data): 
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z","intensity"))
        intensity = []
        cloud_points = []
        for p in pc:
            intensity.append(p[3])
            cloud_points.append(p)
        npc = np.array(cloud_points)
    
        
        '''plt.figure()
        plt.hist(intensity)
        plt.savefig("/constraint_model/images/hist.png")
        plt.close()'''
        
        # Projecting point clouds into bird-eye view image
        #im = birds_eye_point_cloud(npc, side_range=(-100, 100), fwd_range=(-20, 20), res=0.1, saveto="/constraint_model/images/bird_eye_{}.png".format(self.counter))
        # Follow this for lane detction: https://github.com/willshw/lane-detection/blob/master/lane-detection.py
        
        xyz_data = npc
        
        road_xyz = pcl.PointCloud_PointXYZI()
        road_xyz.from_array(npc.astype('float32'))
        
        xs, ys, zs, i = xyz_data[:, 0], xyz_data[:, 1],xyz_data[:, 2], xyz_data[:, 3]
        
        # intensity
        si = sorted(i)[int(0.996*len(i)):int(0.9995*len(i))]
        
        # height
        zrange = max(zs) - min(zs)
        mi = np.median(zs) - 0.005 * zrange
        mx = np.median(zs) + 0.02 * zrange
        road_data = []
        for d in xyz_data:
            if d[2] >= mi and d[2] <= mx:
                road_data.append(d)
        road_data = np.array(road_data)
        rx, ry, rz, ri = road_data.T
        plt.figure()
        plt.hist(rz, 14)
        plt.savefig("/constraint_model/images/hist.png")
        plt.close()
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(rx, ry, ',')
        plt.savefig("/constraint_model/images/road.png")
        plt.close()
 
        
        
        lane_data = []
        mini = min(si)
        maxi = max(si)
        print("{} {}".format(mini, maxi))
        for d in road_data:
            if d[3] >= mini and d[3] <= maxi:
                lane_data.append(d[:-1])

        lane_data = np.array(lane_data)
        lx, ly, lz = lane_data.T

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(rx, ry, ',')
        ax.plot(lx, ly, ',', color='red')
        plt.savefig("/constraint_model/images/lane.png")
        plt.close()
                
        
        
        
        
        

if __name__=='__main__':
    try:
        CheckTopic()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')
        
        
        
