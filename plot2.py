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
import numpy as np

path_to_file = '/model/cars_frent.json'
path_to_save_dir = '/model/plots/'
evaluation = {
    "frenet_data": {},
    "odom_data": {}
}
car_ids = []
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        subData = obj
        for subObj in subData:
            car_id = subObj["car_id"];
            if car_id not in car_ids:
                car_ids.append(car_id)
            if car_id in evaluation["frenet_data"].keys():
                evaluation["frenet_data"][car_id].append(subObj["frenet_data"])
            else:
                evaluation["frenet_data"][car_id] = []
                evaluation["frenet_data"][car_id].append(subObj["frenet_data"]) 
            
            if car_id in evaluation["odom_data"].keys():
                evaluation["odom_data"][car_id].append(subObj["odom_pos"])
            else:
                evaluation["odom_data"][car_id] = []
                evaluation["odom_data"][car_id].append(subObj["odom_pos"]) 


print(car_ids)

#-------------------------car in frenet frame-----------------------

name = path_to_save_dir+"frenet_frame"
plt.figure()
for car in car_ids:
    data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        data["s"].append(fData["s"])
        data["d"].append(fData["d"])
    x = []
    print("car_id: {} and len:{}".format(car, len(data["s"])))
    for index in range(len(data["s"])):
        x.append(0)
    plt.xlim([10, -10])
    y = data["s"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x, y,'--',label=car, color=hex_number)

    x = []
    for d in data["d"]:
        x.append(d)
    y = data["s"]
    plt.plot(x,y,'.',label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"odom_frame"
plt.figure()
for car in car_ids:
    data = {"x":[], "y":[]}
    for fData in evaluation["odom_data"][car]:
        data["x"].append(fData["x"])
        data["y"].append(fData["y"])
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(data["x"], data["y"],'--',label=car, color=hex_number)

plt.ylabel("y")
plt.xlabel("x")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

'''
name = path_to_save_dir+"radius_theta"
plt.figure()
plt.xlabel("theta")
plt.ylabel("radius")
plt.scatter(theta, evaluation['radius'], s=0.50, color='r')
#plt.plot(evaluation['theta'], evaluation['radius'])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"radius_theta_phi"
fig2 = plt.figure(figsize=(10, 10))
ax = fig2.add_subplot(111, projection='3d')
ax.scatter(
        xs=theta,
        ys=evaluation['phi'],
        zs=evaluation['radius'],
        marker='o')
ax.set_xlabel('Theta (rad)')
ax.set_ylabel('Phi (rad)')
ax.set_zlabel('R  (m)')
plt.savefig(name)
plt.close()
'''
'''
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
'''
