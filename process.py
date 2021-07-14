from __future__ import division
import os
import json
import math
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import random
plt.rcParams.update({'figure.max_open_warning': 0})
from datetime import datetime 
import numpy as np
from collections import OrderedDict

class Data:
    def __init__(self, **entries):
        self.__dict__.update(entries)

class Process:
    path_to_dir = '/data/training_0000_bags/results/'

    '''
    path_to_file = '/constraint_model/extracted_data.txt'
    #path_to_file = '/constraint_model/extracted_data_data1.txt'
    #path_to_file = '/constraint_model/extracted_data_gate_south-end_north-end_20180201_070554.0.txt'
    #path_to_file = '/constraint_model/extracted_data_gate_south-end_north-end_20180202_054546.0.txt'
    evaluation = {
        'ego_odom':None,
        'other_objects': None
    }

    # Edo data per second
    ego_data = OrderedDict()

    # This contains the all the objects data under each second
    other_objects_data = OrderedDict()
    
    # This simply contains object_id per second
    other_objects = OrderedDict()

    # It group by object_id and then store all data of the object id in each second
    other_objects_sec = OrderedDict()
    group_ego_by_sec = OrderedDict()

    # This contains all the data under object_id
    group_by_car = OrderedDict()

    # It is mainly for plotting
    car_processed_data = OrderedDict()
   
    deleted_objects = []
    '''
    current_file = None
    def __init__(self):
        
        dirFiles = os.listdir(self.path_to_dir)
        file_list = []
        for _file in dirFiles: #filter out all non jpgs
            if _file.endswith('.txt'):
                file_list.append(_file)
        file_list.sort(key=lambda f: int(filter(str.isdigit, f)))
        file_count = 0
        
        #self.path_to_dir = '/data/training_0000_bags/results/'
        #file_list = ['segment-10153695247769592104_787_000_807_000_with_camera_labels_bag.txt']
        self.path_to_dir = '/constraint_model/data/new_ibeo_data/'
        file_list = ['gate_south-end_north-end_20210528_064001_0_bag.txt']
        
        for _file in file_list:
            print("Current file: {}".format(_file))
            self.current_file = None
            self.evaluation = {
                'ego_odom':None,
                'other_objects': None
            }

            # Edo data per second
            self.ego_data = OrderedDict()

            # This contains the all the objects data under each second
            self.other_objects_data = OrderedDict()

            # This simply contains object_id per second
            self.other_objects = OrderedDict()

            # It group by object_id and then store all data of the object id in each second
            self.other_objects_sec = OrderedDict()
            self.group_ego_by_sec = OrderedDict()

            # This contains all the data under object_id
            self.group_by_car = OrderedDict()

            # It is mainly for plotting
            self.car_processed_data = OrderedDict()

            self.deleted_objects = []

            # Each file iteration
            with open(self.path_to_dir+_file) as json_file:
                data = json.load(json_file)
                self.evaluation['ego_odom'] = data['ego_odom']
                self.evaluation['other_objects'] = data['other_objects']
                self.current_file = os.path.splitext(_file)[0]
            self.evaluate()

    
    def evaluate(self):
        self.car_processed_data['ego'] = {
            'x': [],
            'y': [],
            'theta': []
        }
        
        # Ego odometry
        previous = None
        for item in self.evaluation['ego_odom']:
            data  = Data(**item)
            if data.sec in self.ego_data:
                    self.ego_data[data.sec].append(data)
            else:
                self.ego_data[data.sec] = []
                self.ego_data[data.sec].append(data)
       

        # Storing the plotting data        
        for sec in self.ego_data:
            item = self.ego_data[sec]
            if len(item) > 1: #>1
                data = item[0]
                self.car_processed_data['ego']['x'].append(data.position_x)
                self.car_processed_data['ego']['y'].append(data.position_y)
            else:
                continue
        
        for sec in self.ego_data.keys():
            print("_____")
            print(sec)
            print(self.ego_data[sec][0].position_x)
            print(self.ego_data[sec][0].position_y)
            #for data in self.ego_data[sec]:
            #    print(data.position_x)
        
        
        # Other objects odometry 
        for item in self.evaluation['other_objects']:
            data  = Data(**item)
            if data.sec in self.other_objects_data:
                self.other_objects_data[data.sec].append(data)
                self.other_objects[data.sec].append(data.object_id)
            else:
                self.other_objects_data[data.sec] = []
                self.other_objects_data[data.sec].append(data)
                self.other_objects[data.sec] = []
                self.other_objects[data.sec].append(data.object_id)

            if data.object_id in self.other_objects_sec:
                if data.sec in self.other_objects_sec[data.object_id]:
                    self.other_objects_sec[data.object_id][data.sec].append(data)
                else:
                    self.other_objects_sec[data.object_id][data.sec] = []
                    self.other_objects_sec[data.object_id][data.sec].append(data)
            else:
                self.other_objects_sec[data.object_id] = {}
                self.other_objects_sec[data.object_id][data.sec] = []
                self.other_objects_sec[data.object_id][data.sec].append(data)
        
        #print(self.other_objects.keys())
        #print(self.ego_data.keys())
        for o_id in self.other_objects_sec.keys():
            if len(self.other_objects_sec[o_id].keys()) < 5:
                self.deleted_objects.append(o_id)
        
        # Grouping the data by car
        previous = {}
        for item in self.evaluation['other_objects']:
            data  = Data(**item)
            if data.object_id in self.group_by_car:
                # Logic to capture the data at each second
                diff = 1  
                if previous[data.object_id] is not None:
                    diff = data.sec-previous[data.object_id]  
                
                if diff >= 1:
                    self.group_by_car[data.object_id].append(data)
                    
            else:
                if data.object_id not in self.deleted_objects:
                    self.group_by_car[data.object_id] = []
                    self.group_by_car[data.object_id].append(data)
                
            previous[data.object_id] = data.sec
        
        for car in self.group_by_car.keys():
            self.car_processed_data[car] = {
                'x': [],
                'y': [],
                'theta': []
            }
            item = self.group_by_car[car]
            
            # It is important to remember that yaw rate is the rate of change of yaw. That it is a rate of change of z-axis on the Euler. 
            for data in item:     
                self.car_processed_data[car]['x'].append(data.position_x)
                self.car_processed_data[car]['y'].append(data.position_y)
                
        
        #print(self.group_by_car.keys())
        #print(self.car_processed_data['ego'])
        #self.plot_a_car(car='ego')
        #self.plot_a_car(car=144)
        #cars = [144, 221, 175, 'ego'] # data 1
        #cars = [240, 46, 127, 'ego'] # extracted_data_gate_south-end_north-end_20180201_070554.0
        #cars = [36, 164, 167, 'ego'] # extracted_data_gate_south-end_north-end_20180202_054546.0
        cars = self.group_by_car.keys()
        cars.append('ego')
        self.plot_multiple_car(cars = cars, png_name = 'combined_trajectory')
        
        '''cars = self.extraction_logic()
        cars.append('ego')
        self.plot_multiple_car(cars = cars, png_name = 'combined_trajectory_cut_in')'''
    
    def extraction_logic(self):
        
        # trainting_0000_bags/results/segment-10153695247769592104_787_000_807_000_with_camera_labels.bag is an example of cut out scenario and lane following. 
        # Because the euclidean distance logic extract all cut0in, cut-out and lane -following scenario. Cut-out becuase it was lane following for a whicle beore the cut out.
        # What we need the lane information. 
        
        # CUT-IN scenario
        # LOGIC: 
        # We choose 8 secod as the projected ego dats arequired. For normal lane change, a peper estimated that duration is 8 second. 
        # 1. Loop through ego_data per second
        # 2. Project the ego path for 8 second.
        # 3. We now the the projected path of ego for 8 second.
        # 4. Loop through each second untill the 8 second
        # 5. At each second find the euclidean distance of the vehicle nearby with ego path.
        # 6. If the euclidean distance is closer to zero of any vehicle at any second. then that has the lane change
        
        count = 0 
        lane_changed_car = {}
        for sec in self.ego_data:
            print("Projecting {} second".format(sec))
            if True:
                # This is for getting the projected data for ego motion from sec to sec+8 seconds
                projected_secs = [key for key in range(sec, sec+8)]
                all_ego_data_till_proj = []
                for temp_sec in projected_secs:
                    if temp_sec in self.ego_data:
                        for temp_data in self.ego_data[temp_sec]:
                            all_ego_data_till_proj.append(temp_data)   
                              
                for proj_sec in projected_secs:
                    
                    #Checking if we have any data for other objects in that sec.
                    if proj_sec in self.other_objects:
                        all_cars_at_sec = list(set(self.other_objects[proj_sec]))
                        for car in all_cars_at_sec:
                            if car not in self.deleted_objects:
                                other_data_list = self.group_by_car[car] 
                                for o_data in other_data_list:
                                    
                                    # Loop through ego_s trajectory till 8 second
                                    for e_data in all_ego_data_till_proj:
                                        x1, y1 = e_data.position_x, e_data.position_y
                                        x2, y2 = o_data.position_x, o_data.position_y
                                        
                                        # Finding the euclidean distance is using this formula square root((x2-x1)^2+ (y2-y1)^2)
                                        d = math.sqrt(
                                            (
                                                math.pow((x2-x1), 2)
                                                +
                                                math.pow((y2-y1), 2)
                                            )
                                        )
                                        if d < 0.25:
                                            if car in lane_changed_car:
                                                lane_changed_car[car].append(d)
                                            else:
                                                lane_changed_car[car] = []
                                                lane_changed_car[car].append(d)
                                        
        #cars = [144, 221, 175] 
        '''cars = [843599169]   
        for car in cars:
            print(car)
            print(lane_changed_car[car])
            print("-------------------------------")'''
            
        '''print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        for data in self.group_by_car[843599169]:
           print(data.position_x)
           print(data.position_y)
           print("_____________")'''
            
            
        
        
        # CUT-OUT scenario
        
        
        
        # Overtaking scenario
        
        
        # Lane following
        
        return lane_changed_car.keys()
            
            
    def test_plot(self, cars =  None):
        plt.figure()
        ax = plt.axes(projection='3d')
        z = []
        count = 0
        for data in self.car_processed_data['ego']['x']:
            z.append(count)
            count += 1
        for car in cars:
            x = self.car_processed_data[car]['x']
            y = self.car_processed_data[car]['y']
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('Timesteps')
            ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
  
        name = self.path_to_save_dir+"3d_test"+car
        plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
        plt.savefig(name)
        plt.close()
    def plot_a_car(self, car = None):
        name = self.path_to_save_dir+"trajectory_projection_"+str(car)
        plt.figure()
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'])
        plt.savefig(name)
        plt.close()
    
    def plot_multiple_car(self, cars = None, png_name = None):        
        
        if self.current_file is not None:
            name = self.path_to_dir+self.current_file+"_"+png_name
            print(name)
            plt.figure()
            fig, ax = plt.subplots()
            for car in cars:
                if car == 'ego':
                    ax.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'], label=car, linewidth=3)
                else:
                    ax.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'], label=car)
            plt.xlabel("x(m)")
            plt.ylabel("y(m)")
            plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
            plt.savefig(name)
            plt.close()
                  


if __name__=='__main__':
    p = Process()
