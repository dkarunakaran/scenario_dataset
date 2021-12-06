#!/usr/bin/env python

from __future__ import division
import os
import json
import matplotlib
import carla

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
            

#-------------------------car in frenet frame-----------------------
print(car_ids)

name = path_to_save_dir+"frenet_frame"
plt.figure()
car_ids = [114]
for car in car_ids:
    data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        data["s"].append(fData["s"])
        data["d"].append(fData["d"])
    x = []
    for index in range(len(data["s"])):
        x.append(0)
    plt.xlim([8, -8])
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

#--------------------------------Ego vehicle in frenet frame-----------------------

path_to_file = '/model/ego_frent.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 

name = path_to_save_dir+"ego_frenet_frame"
plt.figure()
data = {"s":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    data["s"].append(fData["s"])
    data["d"].append(fData["d"])
x = []
for index in range(len(data["s"])):
    x.append(0)
plt.xlim([8, -8])
y = data["s"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x, y,'--', color=hex_number)

x = []
for d in data["d"]:
    x.append(d)
y = data["s"]
plt.plot(x,y,'.',label="ego", color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


#--------------------------------Ego vehicle with other vehicle frene frame-----------------------

path_to_file = '/model/ego_frent.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 
name = path_to_save_dir+"frenet_frame_all"
plt.figure()
data = {"s":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    data["s"].append(fData["s"])
    data["d"].append(fData["d"])

x = data["d"]
y = data["s"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x,y,'.',label="ego", color=hex_number)
plt.xlim([8, -8])
print(car_ids)
car_ids = [114]
for car in car_ids:
    car_data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        car_data["d"].append(fData["d"])
        car_data["s"].append(fData["s"])
    x = car_data["d"]
    y = car_data["s"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x,y,'.',label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

'''
#--------------------------------Ego centerline in odom frame-----------------------

path_to_file = '/model/centerline.json'
path_to_save_dir = '/model/plots/'
evaluation = {
    "ego": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data["ego_center"]:
        evaluation["ego"].append(obj)

name = path_to_save_dir+"centerline"
plt.figure()
data = {"x":[], "y":[]}
for fData in evaluation["ego"]:
    data["x"].append(fData["x"])
    data["y"].append(fData["y"])
x = data["x"]
y = data["y"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x, y,'--', color=hex_number)
plt.ylabel("y")
plt.xlabel("x")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''

