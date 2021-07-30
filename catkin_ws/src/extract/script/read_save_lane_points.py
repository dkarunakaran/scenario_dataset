#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import pcl
from rospy.numpy_msg import numpy_msg
import cv2
import numpy as np
from collections import OrderedDict
from std_msgs.msg import Bool
import json
import os

class ReadSaveLanePoints:
    def __init__(self):
        rospy.init_node("ReadSaveLanePoints node")
        path_to_result = rospy.get_param('result')
        path_to_lane_data_dir = path_to_result+"/lane_points/"
        lane_point_create = False
        if not os.path.exists(path_to_result):
            print("{} Not existing......................".format(path_to_result))
            try:
                os.makedirs(path_to_result)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
            if not os.path.exists(path_to_lane_data_dir):
                print("{} Not existing......................".format(path_to_lane_data_dir))
                lane_point_create = True  
        else:
            if not os.path.exists(path_to_lane_data_dir):
                print("{} Not existing......................".format(path_to_lane_data_dir))
                lane_point_create = True
        
        if lane_point_create:
            try:
                os.makedirs(path_to_lane_data_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
            
        self.multi_cloud_points = OrderedDict()
        self.path_lane_data = path_to_lane_data_dir+"lane_data_{}.txt"  
        rospy.Subscriber('/lane_pc', numpy_msg(PointCloud2), self.read_lane_points, queue_size=1) 
        rospy.Subscriber('/signal_shutdown', Bool, self.save_data) 
        rospy.spin()
        
    def read_lane_points(self, data):
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z","intensity"))
        cloud_points = []
        for p in pc:
            cloud_points.append(p)
        xyzi_data = np.array(cloud_points)
        _dict = {}
        _dict['x'], _dict['y'],_dict['z'],_dict['i'] = xyzi_data[:,0].tolist(), xyzi_data[:,1].tolist(), xyzi_data[:,2].tolist(), xyzi_data[:,3].tolist()       
        if data.header.stamp.secs in self.multi_cloud_points:
            self.multi_cloud_points[data.header.stamp.secs].append(_dict)
        else:
            self.multi_cloud_points[data.header.stamp.secs] = []
            self.multi_cloud_points[data.header.stamp.secs].append(_dict)

    def save_data(self, msg):
        if msg.data == True:
            print("Saving to files process started...")
            for key in self.multi_cloud_points.keys():
                with open(self.path_lane_data.format(key), 'w') as outfile:
                    json.dump(self.multi_cloud_points[key], outfile)
            print("Finished saving to files process...")      
        rospy.signal_shutdown("Everything is finishes and shutdown called...")
        

if __name__ == "__main__":
    try:
        ReadSaveLanePoints()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start ReadSaveLanePoints node.')
                
    