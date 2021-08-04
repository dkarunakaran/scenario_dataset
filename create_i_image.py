import os
import json 
import numpy as np
from PIL import Image

class Point:
    def __init__(self, **entries):
        self.__dict__.update(entries)


path_to_dir = '/constraint_model/data/new_ibeo_data/gate_south-end_north-end_20210608_054523.0/lane_points/'
if path_to_dir is not None:
    dirFiles = os.listdir(path_to_dir)
    file_list = []
    for _file in dirFiles: #filter out all non jpgs
        if _file.endswith('.txt'):
            file_list.append(_file)
    file_list.sort(key=lambda f: int(filter(str.isdigit, f)))

#Getting the sec_data
sec_data = []
for _file in file_list:
    path_to_file = path_to_dir+_file
    with open(path_to_file) as json_file:
        json_data = json.load(json_file)
        sec_data.append(json_data['sec'])

sec_data.sort()
sec_start = sec_data[0]
allowed_sec = [sec for sec in range(sec_start,sec_start+8)]
print("allowed sec: {}".format(allowed_sec))

file_count = 0
xyi_cloud = {
    'x': [],
    'y': [],
    'i': []
}
max_x = -100000000
max_y = -100000000
data = []
for _file in file_list:
    path_to_file = path_to_dir+_file
    print("Current file: {}".format(path_to_file))
    with open(path_to_file) as json_file:
        json_data = json.load(json_file)
        if json_data['sec'] in allowed_sec:
            for item in json_data['data']:
                point = Point(**item)
                if len(data)>0:
                    save = True
                    for p in data:
                        if p.x == point.x and p.y == point.y:
                            save = False
                            break
                    if save == True:
                        data.append(point)
                else:
                    data.append(point)
        if json_data['max_x'] >  max_x:
            max_x = json_data['max_x']
        if json_data['max_y'] > max_y:
            max_y = json_data['max_y']
    file_count += 1

#image matrix[row, column], here row is y and column is x.
#image[y,x]
# It should be y as row and x as the column, but for some reason in data x become y and y become x 
img = np.zeros([max_x, max_y, 3], dtype=np.uint8)
for point in data:
    img[point.x, point.y] = [255, 255, 255]
    print("x: {} and y: {}".format(p.x, p.y))

im = Image.fromarray(img)
print("Image is saving..")
im.save('/constraint_model/images/intensity_image.png')

