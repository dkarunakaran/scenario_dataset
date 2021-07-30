import os
import json 

path_to_dir = '/constraint_model/data/new_ibeo_data/gate_south-end_north-end_20210608_054523.0/lane_points/'
if path_to_dir is not None:
    dirFiles = os.listdir(path_to_dir)
    file_list = []
    for _file in dirFiles: #filter out all non jpgs
        if _file.endswith('.txt'):
            file_list.append(_file)
    file_list.sort(key=lambda f: int(filter(str.isdigit, f)))
file_count = 0
for _file in file_list:
    path_to_file = path_to_dir+_file
    print("Current file: {}".format(path_to_file))
    with open(path_to_file) as json_file:
        data = json.load(json_file)
        for pc in data:
            x,y,z,i = pc['x'], pc['y'], pc['z'], pc[i]
            print(np.array([x,y,z,i]))
    
    file_count += 1
    if file_count > 2:
        break
    