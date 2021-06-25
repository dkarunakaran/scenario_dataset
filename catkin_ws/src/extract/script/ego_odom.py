#!/usr/bin/env python

from __future__ import division
from data import Data
import tf

class EgoOdom:
    
    item = []
    group_item = {}
    item_timestamp = []

    def __init__(self):
        pass
    
    def put(self, timestamp= None, msg = None):
        self.data = Data()
        self.data.long_vel = msg.twist.twist.linear.x
        self.data.lat_vel = msg.twist.twist.linear.y
        self.data.positon_x = msg.pose.pose.position.x
        self.data.positon_y = msg.pose.pose.position.y
        self.data.orientation_x = msg.pose.pose.orientation.x
        self.data.orientation_y = msg.pose.pose.orientation.y
        self.data.orientation_z = msg.pose.pose.orientation.z
        self.data.orientation_w = msg.pose.pose.orientation.w
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.data.yaw = euler[2]
        self.item_timestamp.append(timestamp.secs)
        self.item.append(self.data)
        
        if timestamp.secs in self.group_item:
            self.group_item[timestamp.secs].append(self.data)
        else:
            self.group_item[timestamp.secs] = []
            self.group_item[timestamp.secs].append(self.data)
        
    def get(self, name):
        if name in self.item.keys():
            return self.item[name]
        else:
            return None
    
    def show(self):
        print(self.item)