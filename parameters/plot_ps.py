import os
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import json
import math
import numpy as np

#-----------------------------------esmini data------------------------------
path_to_save_dir = 'plots/'
_file = 'parameter_space/cutin_space.json'
with open(_file) as f:
    data = json.loads(f.read())

print("total number of scenarios: {}".format(len(data['data']['param_cut_triggering_dist'])))


name = path_to_save_dir+"histo_cut_triggering_dist"
plt.figure()
plt.hist(data['data']['param_cut_triggering_dist'])
plt.savefig(name)
plt.close()

