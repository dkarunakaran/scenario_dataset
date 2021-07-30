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
from PIL import Image


class CheckTopic:
    counter = 0
    def __init__(self):
        rospy.init_node("Starting the CheckTopic code")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer) 
        #rospy.Subscriber('/ibeo/lidar/static', numpy_msg(PointCloud2), self.check_topic) 
        #rospy.Subscriber('/lidar_pointcloud/top', numpy_msg(PointCloud2), self.check_topic, queue_size=20) 
        #rospy.Subscriber('/pointcloud_transformer/output_pcl2', numpy_msg(PointCloud2), self.check_topic)
        #rospy.Subscriber('/velodyne_points', PointCloud2, self.test)
        #rospy.Subscriber('/lidar_pointcloud/top', PointCloud2, self.test)
        rospy.spin()
        
    
    def check_topic(self, data): 
        if data.header.stamp.secs in self.sec_watcher:
            pass
        else:
            self.sec_watcher.append(data.header.stamp.secs)
            print("**********************count: {}*********************".format(self.counter))
            pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z","intensity"))
            cloud_points = []
            for p in pc:
                cloud_points.append(p)
            xyzi_data = np.array(cloud_points)        
            xyzi_data = self.elevation_filter(xyzi_data)
            
            saveto = "/constraint_model/images/bird_eye_{}.png".format(self.counter)
            # Projecting point clouds into bird-eye view image
            xyzi_data, image = birds_eye_point_cloud(xyzi_data, side_range=(-10, 10), fwd_range=(0, 30), res=0.1, _type='intensity', saveto=saveto)
            
            im = Image.fromarray(image)
            print("Image is saving..")
            im.save(saveto)
            

            self.counter += 1
        
        
    def elevation_filter(self, pts):
        # filter with elevation
        
        print('Before filter, shape {}'.format(np.array(pts).shape))
        result = []
        mean = np.mean(pts[:, 2])
        std = np.std(pts[:, 2])
        threshold = mean
        for point in pts:
            if point[2] < threshold:
                result.append(point)
                
        print('After filter, shape {}'.format(np.array(result).shape))
        
        return np.array(result)
        
    
if __name__=='__main__':
    try:
        CheckTopic()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')
        
        
        
