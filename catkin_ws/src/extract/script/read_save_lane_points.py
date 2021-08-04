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
from lane_points_msg.msg import LanePoints
import json
import os

class Point:
    def __init__(self,x_init,y_init):
        self.x = x_init
        self.y = y_init


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
        rospy.Subscriber('/lane_points', LanePoints, self.read_lane_points, queue_size=1) 
        rospy.Subscriber('/signal_shutdown', Bool, self.save_data) 
        rospy.spin()
        
    def read_lane_points(self, msg):
        
        sec = msg.header.stamp.secs
        store = {}
        store['sec'] = sec
        store['max_x'] = msg.max_x
        store['max_y'] = msg.max_y
        store['data'] = []
        for index in range(len(msg.x.data)):
            store['data'].append(Point(msg.x.data[index], msg.y.data[index]).__dict__)
        self.multi_cloud_points[sec] = store    
        

    def save_data(self, msg):
        if msg.data == True:
            print("Saving to files process started...")
            for key in self.multi_cloud_points.keys():
                print(key)
                with open(self.path_lane_data.format(key), 'w') as outfile:
                    json.dump(self.multi_cloud_points[key], outfile)
            print("Finished saving to files process...")      
        rospy.signal_shutdown("Everything is finishes and shutdown called...")
        

if __name__ == "__main__":
    try:
        ReadSaveLanePoints()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start ReadSaveLanePoints node.')
                
    
