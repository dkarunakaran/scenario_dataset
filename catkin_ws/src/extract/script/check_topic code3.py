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

from sklearn.preprocessing import StandardScaler

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
        
        filtered_point = self.filter1(xyz_data)
        
        X = StandardScaler().fit_transform(filtered_point)
        db = DBSCAN(eps=0.1, min_samples=20).fit(X)
        labels = db.labels_
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)
        #print(n_clusters_)
        #print(n_noise_)


        width = 4  # width of line markings
        clusters = []

        # Visualize
        unique_labels = set(labels)
        colors = [plt.cm.Spectral(each)
                for each in np.linspace(0, 1, len(unique_labels))]
        for k, col in zip(unique_labels, colors):
            if k == -1:
                continue

            class_member_mask = (labels == k)
            xy = X[class_member_mask & core_samples_mask]
            #print(xy.shape)
            plt.scatter(xy[:, 0], xy[:, 1], s=1)  # c=tuple(col),

            # two ends
            x1, y1, _, _ = np.amax(xy,axis=0)
            x2, y2, _, _ = np.amin(xy,axis=0)

            k = (y2-y1) / (x2-x1)
            b = y1 - k*x1
            clusters.append((k, b, x1, x2))

        
        plt.savefig("/constraint_model/images/cluster_{}.png".format(self.counter))
        plt.close()
    

        
        
        self.counter += 1
        
        
        
    def filter1(self, pts):
        # filter the data with intensity

        intensity = pts[:, 3]
        mean = np.mean(intensity)
        std = np.std(intensity)
        
        minus = abs((max(intensity)*5)/100)

        threshold = (mean + std) - minus#- 10  # adjust this value to see different results
        print('threshold is {}'.format(threshold))

        filtered_points = []

        for point in pts:
            if point[3] > threshold:
                filtered_points.append(point)
        print('after filter 1 shape {}'.format(np.array(filtered_points).shape))
        return np.array(filtered_points)


    def filter2(self, pts):
        with open('results/trajectory.xyz') as tra:
            data = tra.readline()
            trajectory = []
            while data:
                trajectory.append([float(x) for x in data.split()])
                data = tra.readline()

        trajectory = np.array(trajectory)[:, :2]
        # print(f'trajectory is {trajectory}')

        threshold = 20  # in meters. adjust this to see different results

        result = []
        for point in pts:
            dist = min(np.sum((trajectory - point[:2]) ** 2, axis=1)) ** 0.5
            if dist < threshold:
                result.append(point)
        print('after filter 2 shape {}'.format(np.array(result).shape))
        return np.array(result)


    def filter3(self, pts):

        # filter with elevation

        result = []

        mean = np.mean(pts[:, 2])
        std = np.std(pts[:, 2])

        threshold = mean

        for point in pts:
            if point[2] < threshold:
                result.append(point)

        print('after filter 3 shape {}'.format(np.array(result).shape))
        return np.array(result)
                    
        
        
        
        
        

if __name__=='__main__':
    try:
        CheckTopic()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')
        
        
        
