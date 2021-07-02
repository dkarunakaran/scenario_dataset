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

class Data:
    def __init__(self, **entries):
        self.__dict__.update(entries)

class Process:
    path_to_file = '/constraint_model/extracted_data_data1.txt'
    #path_to_file = '/constraint_model/extracted_data_gate_south-end_north-end_20180201_070554.0.txt'
    evaluation = {
        'ego_odom':None,
        'ibeo_objects': None
    }

    # Edo data per second
    ego_data = {}

    # This contains the all the objects data under each second
    other_objects_data = {}
    
    # This simply contains object_id per second
    other_objects = {}

    # It group by object_id and then store all data of the object id in each second
    other_objects_sec = {}
    group_ego_by_sec = {}

    # This contains all the data under object_id
    group_by_car = {}

    # It is mainly for plotting
    car_processed_data = {}
    
    def __init__(self):
        
        with open(self.path_to_file) as json_file:
            data = json.load(json_file)
            self.evaluation['ego_odom'] = data['ego_odom']
            self.evaluation['ibeo_objects'] = data['ibeo_objects']
            self.path_to_save_dir = '/constraint_model/plots/'
    
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
            if len(item) > 1:
                data = item[0]
                self.car_processed_data['ego']['x'].append(data.position_x)
                self.car_processed_data['ego']['y'].append(data.position_y)
            else:
                continue
        
        # Other objects odometry 
        for item in self.evaluation['ibeo_objects']:
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

        
        # Grouping the data by car
        previous = {}
        for item in self.evaluation['ibeo_objects']:
            data  = Data(**item)
            if data.object_id in self.group_by_car:
                # Logic to capture the data at each second
                diff = 1  
                if previous[data.object_id] is not None:
                    diff = data.sec-previous[data.object_id]  
                
                if diff >= 1:
                    self.group_by_car[data.object_id].append(data)
                    
            else:
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
        #cars = [128, 16, 240, 196, 5, 166, 200, 252, 110, 46, 176, 168, 178, 53, 218, 127, 28, 159, 'ego']
        cars = [144, 221, 175, 'ego']
        self.plot_multiple_car(cars = cars)
        
        self.extraction_logic()
    
    def extraction_logic(self):
        
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
            if True:
                # This is for getting the projected data for ego motion
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
                            if True:
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
                                        if d < 1:
                                            if car in lane_changed_car:
                                                lane_changed_car[car].append(d)
                                            else:
                                                lane_changed_car[car] = []
                                                lane_changed_car[car].append(d)
                                        
                                    
                                '''
                                if(len(other_data_list) > 1):
                                    first = 0
                                    second = len(other_data_list)-1
                                    x1, y1 = other_data_list[first].position_x, other_data_list[first].position_y
                                    x2, y2 = other_data_list[second].position_x, other_data_list[second].position_y
                                    # Finding the slope m = (y2-y1)/x2-x1)
                                    m = (y2-y1)/(x2-x1)
                                    #print(m)
                                    b = y2-m*x2
                                    #print(b)
                                    
                                    # Finding the middle point in the line:
                                    x_mid = (x1+x2)/2     
                                    y_mid = (y1+y2)/2
                                    print("(x,y) = ({},{})".format(x_mid,y_mid))
                                    
                                    # Now I need to find the perpendicular line to the created line
                                    # Negative reciprocal of the slope is the slope of the perpendicular line:
                                    m_p = np.reciprocal(m)*-1 
                                    # Finding the slope m = (y2-y1)/x2-x1)
                                    # Now we need to find the perpendicular line that passes through (x,y) on the other line
                                    # Based on this site : https://www.mathsisfun.com/algebra/line-parallel-perpendicular.html. We can find 
                                    # the equation of the perpendicular line that passess through (x,y)
                                    # call the point(x_mid, y_mid) to (x1p,y1p)
                                    x1p, y1p = x_mid, y_mid
                                    # The equation now is : (y-y1p) = m_p(x - x1p)
                                    # y = (m_p*x - m_p*x1p)+y1p
                                    print(m_p)         
                                else:
                                    print(other_data_list[0].position_x)
                                    print(other_data_list[0].position_y)
                                '''
        #cars = [144, 221, 175] 
        cars = [221]   
        for car in cars:
            print(car)
            print(lane_changed_car[car])
            print("-------------------------------")
        
        # CUT-OUT scenario
        
        
        
        # Overtaking scenario
            
            
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
    
    def plot_multiple_car(self, cars = None):        
        name = self.path_to_save_dir+"combined_trajectory"
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
    p.evaluate()
