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
    'ego':{
        'long_vel': [],
        'rss_long_no_front_vehicle': [],
        'timesteps': [],
        'accel': [],
        'rss_long_with_vehicle': []
    },
    'other_car': {
        'long_vel': [],
        'lat_vel': [],
        'long_accel': [],
        'rel_pos_x': [],
        'rel_pos_y': [],
        'yaw': []
    },
    'tracked_objects': None,
    'closest_cars': None,
    'all_objects_timesteps': None
}
path_to_file = '/constraint_model/extracted_data.txt'
path_to_save_dir = '/constraint_model/plots/'
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for index in range(len(data['ego']['long_vel'])):
        km_hr = (data['ego']['long_vel'][index]*18)/5
        evaluation['ego']['long_vel'].append(km_hr)
        evaluation['ego']['timesteps'].append(index)
    
    for index in range(len(data['ego']['rss_long_no_front_vehicle'])):
        evaluation['ego']['rss_long_no_front_vehicle'].append(data['ego']['rss_long_no_front_vehicle'][index])
        
    for index in range(len(data['ego']['rss_long_with_vehicle'])):
        evaluation['ego']['rss_long_with_vehicle'].append(data['ego']['rss_long_with_vehicle'][index])
    
    for index in range(len(data['ego']['accel'])):
        evaluation['ego']['accel'].append(data['ego']['accel'][index])
        
    for index in range(len(data['other_car']['long_vel'])):
        evaluation['other_car']['long_vel'].append(data['other_car']['long_vel'][index])
        
    for index in range(len(data['other_car']['lat_vel'])):
        evaluation['other_car']['lat_vel'].append(data['other_car']['lat_vel'][index])
        
    for index in range(len(data['other_car']['long_accel'])):
        evaluation['other_car']['long_accel'].append(data['other_car']['long_accel'][index])
        
    for index in range(len(data['other_car']['rel_pos_x'])):
        evaluation['other_car']['rel_pos_x'].append(data['other_car']['rel_pos_x'][index])
    
    for index in range(len(data['other_car']['rel_pos_y'])):
        evaluation['other_car']['rel_pos_y'].append(data['other_car']['rel_pos_y'][index])
        
    for index in range(len(data['other_car']['yaw'])):
        evaluation['other_car']['yaw'].append(data['other_car']['yaw'][index])
        
    evaluation['tracked_objects'] = data['tracked_objects']
    evaluation['closest_cars'] = data['closest_cars']
    evaluation['all_objects_timesteps'] = data['all_objects_timesteps']
    
    
    

'''
name = path_to_save_dir+"histo"
plt.figure()
plt.hist(evaluation['ego']['long_vel'], bins='auto')
plt.title("Histogram of Longitudinal velocity")
plt.xlabel("Longitudinal velocity(km/hr)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

name = path_to_save_dir+"histo_rss_long_no_front_vehicle"
plt.figure()
plt.hist(evaluation['ego']['rss_long_no_front_vehicle'], bins='auto')
plt.savefig(name)
plt.close()


legend = []
name = path_to_save_dir+"long_vel_timesteps"
plt.figure(figsize=(8,6))
plt.title("Longitudinal velocity vs timesteps")
plt.xlabel("Timesteps")
plt.ylabel("Longitudinal velocity(km/hr)")
plt.plot(evaluation['ego']['timesteps'], evaluation['ego']['long_vel'], linewidth=2) 
#plt.legend(legend)
plt.savefig(name)
plt.close()



legend = []
name = path_to_save_dir+"long_vs_rss_long"
plt.figure(figsize=(8,6))
plt.title("Longitudinal RSS with front vehicle is moving at 15m/s(72km/hr)")
plt.xlabel("Longitudinal velocity(km/hr)")
plt.ylabel("Longitudinal RSS(m)")
plt.plot(evaluation['ego']['long_vel'], evaluation['ego']['rss_long_no_front_vehicle'], linewidth=2) 
#plt.legend(legend)
plt.savefig(name)
plt.close()


x = evaluation['ego']['timesteps']
y = evaluation['ego']['long_vel']
z = evaluation['ego']['rss_long_no_front_vehicle'] 

name = path_to_save_dir+"3d"
plt.figure(figsize=(6,6))
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('Long_vel(km/hr)')
ax.set_zlabel('Long_rss(m)')
plt.title("3D plot with no front vehicle")
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.savefig(name)
plt.close()
'''
x = evaluation['ego']['timesteps']
y = evaluation['ego']['long_vel']
z = evaluation['ego']['rss_long_with_vehicle']
print(len(x))
print(len(y))
print(len(z))

name = path_to_save_dir+"3d"
plt.figure(figsize=(6,6))
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('Long_vel(km/hr)')
ax.set_zlabel('Long_rss(m)')
plt.title("3D plot with front vehicle")
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.savefig(name)
plt.close()

'''

name = path_to_save_dir+"accel_histo"
plt.figure()
plt.hist(evaluation['ego']['accel'], bins='auto', range=(-5,5))
plt.title("Histogram of Longitudinal acceleration")
plt.xlabel("Longitudinal acceleration(m/s2)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

'''
'''
name = path_to_save_dir+"other_long_histo"
plt.figure()
plt.hist(evaluation['other_car']['long_vel'], bins='auto')
plt.title("Histogram of Longitudinal vel of other_cars")
plt.xlabel("Longitudinal velocity(m/s)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

name = path_to_save_dir+"other_lat_histo"
plt.figure()
plt.hist(evaluation['other_car']['lat_vel'], bins='auto')
plt.title("Histogram of lateral vel of other_cars")
plt.xlabel("Lateral velocity(m/s)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

name = path_to_save_dir+"long_accel_histo_other"
plt.figure()
plt.hist(evaluation['other_car']['long_accel'], bins='auto', range=(-20,20))
plt.title("Histogram of Longitudinal acceleration of other_cars")
plt.xlabel("Longitudinal acceleration(m/s2)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

name = path_to_save_dir+"other_rel_pos_y_histo"
plt.figure()
plt.hist(evaluation['other_car']['rel_pos_y'], bins='auto')
plt.title("Histogram of relative pose in y direction of other_cars")
plt.xlabel("Relative pos y(m)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()

name = path_to_save_dir+"other_rel_pos_x_histo"
plt.figure()
plt.hist(evaluation['other_car']['rel_pos_x'], bins='auto')
plt.title("Histogram of relative pose in x direction of other_cars")
plt.xlabel("Relative pos x(m)")
plt.ylabel("Frequency")
plt.savefig(name)
plt.close()



name = path_to_save_dir+"long_vel_comparison"
plt.figure()
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
r = lambda: random.randint(0,255)
for key in objects.keys():
    long_vel = []
    for index in range(len(objects[key])):
        long_vel.append(objects[key][index]['long_vel'])
        
    y = long_vel
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(y, label=key, color=hex_number)
plt.ylabel("Longitudinal velocity(m/s)")
plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()



name = path_to_save_dir+"lat_vel_comparison"
plt.figure()
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
#r = lambda: random.randint(0,255)
for key in objects.keys():
    long_vel = []
    for index in range(len(objects[key])):
        long_vel.append(objects[key][index]['lat_vel'])
        
    y = long_vel
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(y, label=key, color=hex_number)
plt.ylabel("Lateral velocity(m/s)")
plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"rel_lat_dist_comparison"
plt.figure()
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
#r = lambda: random.randint(0,255)
for key in objects.keys():
    long_vel = []
    for index in range(len(objects[key])):
        long_vel.append(objects[key][index]['rel_pos_y'])
        
    y = long_vel
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(y, label=key, color=hex_number)
plt.ylabel("Relative lateral position(m)")
plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"rel_long_dist_comparison"
plt.figure()
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
#r = lambda: random.randint(0,255)
for key in objects.keys():
    long_vel = []
    for index in range(len(objects[key])):
        long_vel.append(objects[key][index]['rel_pos_x'])
        
    y = long_vel
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(y, label=key, color=hex_number)
plt.ylabel("Relative longitudinal position(m)")
plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"other_long_accel_comparison"
plt.figure()
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
#r = lambda: random.randint(0,255)
for key in objects.keys():
    long_vel = []
    for index in range(len(objects[key])):
        long_vel.append(objects[key][index]['long_accel'])
    y = long_vel
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(y, label=key, color=hex_number)
plt.ylabel("Other cars' acceleration(m/s2)")
plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''

'''
# Not working
name = path_to_save_dir+"3d_rel_dist_comparison"
plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('Long_vel(km/hr)')
ax.set_zlabel('Long_rss(m)')
x = evaluation['ego']['timesteps']
objects = evaluation['tracked_objects']
#r = lambda: random.randint(0,255)
for key in objects.keys():
    long_pos = []
    lat_pos = []
    for index in range(len(objects[key])):
        lat_pos.append(objects[key][index]['rel_pos_y'])
        
    y = lat_pos
    #hex_number = '#%02X%02X%02X' % (r(),r(),r())
    
    for index in range(len(objects[key])):
        long_pos.append(objects[key][index]['rel_pos_y'])
    
    z = long_pos
    ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
    #plt.plot(y, label=key, color=hex_number)
#plt.ylabel("Relative lateral position(m)")
#plt.xlabel("Timesteps(s)")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''
from datetime import datetime
x = []
y = []
for index in range(len(evaluation['all_objects_timesteps'])):
    x.append(datetime.fromtimestamp(evaluation['all_objects_timesteps'][index]))
for index in range(len(evaluation['closest_cars']['144'])):
    #x.append(index)
    y.append(evaluation['closest_cars']['144'][index]['yaw'])
name = path_to_save_dir+"yaw_histo_144"
plt.figure()
plt.plot_date(x, y[:len(x)])
plt.xlabel("Timesteps")
plt.ylabel("yaw(rad)")
plt.ylim((-0.2,0.2))

plt.savefig(name)
plt.close()
