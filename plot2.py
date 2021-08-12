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

evaluation = {
    'radius': [],
    'theta': [],
    'phi': []
}
path_to_file = '/constraint_model/data/radius_theta_phi.txt'
path_to_save_dir = '/constraint_model/plots/'
with open(path_to_file) as json_file:
    data = json.load(json_file)
    evaluation['radius'] = data['radius'] 
    evaluation['theta'] = data['theta'] 
    evaluation['phi'] = data['phi'] 

#print(evaluation['theta'])
print(len(evaluation['theta']))
print(len(evaluation['phi']))

#angles = np.array(evaluation['theta'])
#theta = (2*np.pi + angles) * (angles < 0) + angles*(angles > 0)
#theta = theta.tolist()
theta = evaluation['theta']

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
