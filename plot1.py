#!/usr/bin/env python

from __future__ import division
import os
import json
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import random
plt.rcParams.update({'figure.max_open_warning': 0})

evaluation = {
    'ego_odom':{
        'x': [],
        'y': [],
        'theta': []
    },
    'other_odom': {}

}
path_to_file = '/constraint_model/extracted_data.txt'
path_to_save_dir = '/constraint_model/plots/'
with open(path_to_file) as json_file:
    data = json.load(json_file)
    '''for index in range(len(data['ego']['long_vel'])):
        km_hr = (data['ego']['long_vel'][index]*18)/5'''
    
    for index in range(len(data['ego_odom']['x'])):
        evaluation['ego_odom']['x'].append(data['ego_odom']['x'][index])
        
    for index in range(len(data['ego_odom']['y'])):
        evaluation['ego_odom']['y'].append(data['ego_odom']['y'][index])
        
    for index in range(len(data['ego_odom']['theta'])):
        evaluation['ego_odom']['theta'].append(data['ego_odom']['theta'][index])
        
    for key in data['other_odom'].keys():
        key = str(key)
        evaluation['other_odom'][key] = {
            'x': [],
            'y': []
        }
        for item in data['other_odom'][key]['x']:
            evaluation['other_odom'][key]['x'].append(item)
        for item in data['other_odom'][key]['y']:
            evaluation['other_odom'][key]['y'].append(item)
            
        
print(evaluation['other_odom'].keys())

name = path_to_save_dir+"trajectory_projection_ego"
plt.figure()
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.plot(evaluation['ego_odom']['x'], evaluation['ego_odom']['y'])
plt.savefig(name)
plt.close()

car = '144'
name = path_to_save_dir+"trajectory_projection_"+str(car)
plt.figure()
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.plot(evaluation['other_odom']['144']['x'], evaluation['other_odom']['144']['y'])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"combined_trajectory"
plt.figure()
fig, ax = plt.subplots()
ax.plot(evaluation['other_odom']['144']['x'], evaluation['other_odom']['144']['y'])
ax.plot(evaluation['other_odom']['123']['x'], evaluation['other_odom']['123']['y'])
ax.plot(evaluation['ego_odom']['x'], evaluation['ego_odom']['y'])
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()