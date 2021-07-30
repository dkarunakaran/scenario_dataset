import os
import json 
import numpy as np

path_to_dir = '/constraint_model/data/new_ibeo_data/gate_south-end_north-end_20210608_054523.0/lane_points/'
if path_to_dir is not None:
    dirFiles = os.listdir(path_to_dir)
    file_list = []
    for _file in dirFiles: #filter out all non jpgs
        if _file.endswith('.txt'):
            file_list.append(_file)
    file_list.sort(key=lambda f: int(filter(str.isdigit, f)))
file_count = 0
xyzi_cloud = []
for _file in file_list:
    path_to_file = path_to_dir+_file
    print("Current file: {}".format(path_to_file))
    with open(path_to_file) as json_file:
        data = json.load(json_file)
        for pc in data:
            xyzi_data = np.array([pc['x'], pc['y'], pc['z'], pc['i']])
            xyzi_cloud.append(xyzi_data)
    
    file_count += 1
    if file_count > 2:
        break
    
max_x = -100000000
max_y = -100000000
min_x = 100000000
min_y = 100000000
for xyzi_data in xyzi_cloud:
    x,y,y,i  = xyzi_data
    temp_min_x = min(x)
    temp_min_y = min(y)
    temp_max_x = max(x)
    temp_max_y = max(y)
    
    if temp_min_x < min_x:
        min_x = temp_min_x
    if temp_min_y < min_y:
        min_y = temp_min_y
    
    if temp_max_x >  max_x:
        max_x = temp_max_x
    if temp_max_y > max_y:
        max_y = temp_max_y
        
image_max_x = max_x - min_x + 2;
image_max_y = max_y - max_y + 2;

print(min_y)
print(max_y)