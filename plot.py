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
        'rss_long_with_vehicle': [],
        'yaw': []
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
    'all_objects_timesteps': None,
    'rss':{
        'cut-in': None
    }
}
path_to_file = '/constraint_model/extracted_data.txt'
path_to_save_dir = '/constraint_model/plots/'
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for index in range(len(data['ego']['long_vel'])):
        km_hr = (data['ego']['long_vel'][index]*18)/5
        evaluation['ego']['long_vel'].append(km_hr)
        evaluation['ego']['timesteps'].append(index)
    
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
        
    for index in range(len(data['ego']['yaw'])):
        evaluation['ego']['yaw'].append(data['ego']['yaw'][index])
    
    
    evaluation['rss']['cut-in'] = data['rss']['cut-in']
    evaluation['tracked_objects'] = data['tracked_objects']
    evaluation['closest_cars'] = data['closest_cars']
    evaluation['all_objects_timesteps'] = data['all_objects_timesteps']
    





name = path_to_save_dir+"histo_yaw_ego"
plt.figure()
plt.hist(evaluation['ego']['yaw'], bins='auto')
plt.savefig(name)
plt.close()

'''
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

x = evaluation['ego']['timesteps']
y = evaluation['ego']['long_vel']
z = evaluation['ego']['rss_long_with_vehicle']
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

from datetime import datetime

plt.figure()
fig, ax = plt.subplots()
cars = evaluation['rss']['cut-in'].keys()
print(cars)
#cars = ['144', '73', '11', '212', '23', '83', '112', '1', '122', '123', '73', '8', '235', '175', '249', '172']
cars = ['202', '78', '205', '42', '53', '109', '5', '166', '85']
for car in cars:
    x = []
    y = []
    for index in range(len(evaluation['rss']['cut-in'][car])):
        x.append(datetime.fromtimestamp(float(evaluation['rss']['cut-in'][car][index]['timestamp'])))
        y.append(evaluation['rss']['cut-in'][car][index]['rel_pos_y']) 
    r = lambda: random.randint(0,255)  
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    ax.plot(x, y, label=car, color=hex_number)
plt.xlabel("Time")
plt.ylabel("rel_pos_y")
#plt.ylim((-0.2,0.2))
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
#name = path_to_save_dir+"yaw_histo_all"
name = path_to_save_dir+"rel_pos_y_all"
plt.savefig(name)
plt.close()


for car in cars:
    x = []
    y = []
    for index in range(len(evaluation['rss']['cut-in'][car])):
        x.append(datetime.fromtimestamp(float(evaluation['rss']['cut-in'][car][index]['timestamp'])))
        y.append(evaluation['rss']['cut-in'][car][index]['yaw']) 
    r = lambda: random.randint(0,255)  
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    ax.plot(x, y, label=car, color=hex_number)
plt.xlabel("Time")
plt.ylabel("Yaw(rad)")
#plt.ylim((-0.2,0.2))
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
#name = path_to_save_dir+"yaw_histo_all"
name = path_to_save_dir+"yaw_all"
plt.savefig(name)
plt.close()


#-----------------------------------------------
'''
car = '78'

name = path_to_save_dir+"rel_pos_y_"+car
x = []
y = []
plt.figure()
plt.xlabel("Timesteps")
plt.ylabel("rel_pos_y")
plt.plot(x, y)
plt.savefig(name)
plt.close()

# 3D comaprison for yaw and rel_pos_y
x = []
y = []
z = []
for index in range(len(evaluation['closest_cars'][car])):
    x.append(index)
    y.append(evaluation['closest_cars'][car][index]['yaw'])   
    z.append(evaluation['closest_cars'][car][index]['rel_pos_y'])

# Not working
name = path_to_save_dir+"3d_rel_yaw_dist_comparison_"+car
plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('yaw')
ax.set_zlabel('rel_pos_y')
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


from datetime import datetime
x = []
y = []
z = []
for index in range(len(evaluation['closest_cars'][car])):
    x.append(index)
    y.append(evaluation['closest_cars'][car][index]['rel_pos_x'])   
    z.append(evaluation['closest_cars'][car][index]['rel_pos_y'])

# Not working
name = path_to_save_dir+"3d_rel_dist_comparison_"+car
plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('rel_pos_x')
ax.set_zlabel('rel_pos_y')
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''
#---------------------------------------------------------
cars = evaluation['closest_cars'].keys()
#print(evaluation['rss']['cut-in'])

car ='78'
# 3D comaprison for yaw and rel_pos_y
x = []
y = []
z = []
for index in range(len(evaluation['rss']['cut-in'][car])):
    x.append(index)
    y.append(evaluation['rss']['cut-in'][car][index]['rel_pos_y'])   
    z.append(evaluation['rss']['cut-in'][car][index]['lat_rss'])


name = path_to_save_dir+"3d_rel_pos_y_lat_rss_comparison_"+car
plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('rel_pos_y')
ax.set_zlabel('lat_rss')
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

# 3D comaprison for long_rss and rel_pos_x
x = []
y = []
z = []
for index in range(len(evaluation['rss']['cut-in'][car])):
    x.append(index)
    y.append(evaluation['rss']['cut-in'][car][index]['rel_pos_x'])   
    z.append(evaluation['rss']['cut-in'][car][index]['long_rss'])
name = path_to_save_dir+"3d_rel_pos_x_long_rss_comparison_"+car
plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('Timesteps')
ax.set_ylabel('rel_pos_x')
ax.set_zlabel('long_rss')
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
    


name = path_to_save_dir+"lat_vel_"+car
x = []
y = []
for index in range(len(evaluation['rss']['cut-in'][car])):
    x.append(index)
    y.append(evaluation['rss']['cut-in'][car][index]['lat_vel']) 
plt.figure()
plt.xlabel("Timesteps")
plt.ylabel("lat_vel")
plt.plot(x, y)
plt.savefig(name)
plt.close()