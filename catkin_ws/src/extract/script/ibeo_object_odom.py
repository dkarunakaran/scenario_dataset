#!/usr/bin/env python

from __future__ import division
from data import Data
import tf

class IbeoObjectOdom:
    
    item = []
    group_item = {}
    group_objects = {}
    item_timestamp = []
    group_by_car = {}

    def __init__(self):
        pass
    
    def put(self, timestamp= None, msg = None):
        self.data = Data()
        self.data.object_id = msg['object_id']
        self.data.linear_x = msg['linear_x']
        self.data.linear_y = msg['linear_y']
        self.data.linear_z = msg['linear_z']
        self.data.position_x = msg['position_x']
        self.data.position_y = msg['position_y']
        self.data.position_z = msg['position_z']
        self.data.roll = msg['roll']
        self.data.pitch = msg['pitch']
        self.data.yaw = msg['yaw']
        self.data.timestamp_sec = timestamp.secs
        self.data.timestamp_nsec = timestamp.nsecs
        self.item_timestamp.append(timestamp.secs)
        self.item.append(self.data)
        if timestamp.secs in self.group_item:
            self.group_item[timestamp.secs].append(self.data)
        else:
            self.group_item[timestamp.secs] = []
            self.group_item[timestamp.secs].append(self.data)
            
        if self.data.object_id in self.group_by_car:
            self.group_by_car[self.data.object_id].append(self.data)
        else:
            self.group_by_car[self.data.object_id] = []
            self.group_by_car[self.data.object_id].append(self.data)
        
        
    def get(self, name):
        if name in self.item.keys():
            return self.item[name]
        else:
            return None
    
    def show(self):
        print(self.item)