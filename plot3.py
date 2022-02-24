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

x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
y = [10, 12, 13, 13.5, 14.5, 15.5, 16.5, 17, 17.5, 18, 19, 20]
#y = [5, 5.5, 6, 6.5, 7, 7.5, 8, 9, 10, 10.5, 11, 11.5]


path_to_save_dir = '/model/plots/'
name = path_to_save_dir+"ego_frenet_frame_s_t"
plt.figure()
plt.plot(x,y)
plt.ylabel("speed")
plt.xlabel("sec")
plt.savefig(name)
plt.close()




