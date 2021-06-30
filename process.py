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
        #cars = [144, 'ego']
        #self.plot_multiple_car(cars = cars)
        
        self.extraction_logic()
    
    def extraction_logic(self):
        
        # CUT-IN scenario
        # LOGIC: 
        # 1. Loop through ego_data per second
        #   a. find all the vehicle nearby ego at that second
        #   b. find the trajectory each vehicle for next 8 second that meet with eg's trajectory for the next 8 second.
        #   c. if there is any vehicle's trajectory is close(y distance is closer) at nth second, then nsecond-8 is the starting point of the scenaario extraction.
        # There is a problem with "y dsitance is closer" logic as any vehicle was one the same lane can be detected as well.But it's wrong.
        #
        #

        for sec in self.ego_data:
            
            #Find the vehicle in that sec
            if sec in self.other_objects:
                all_cars_at_sec = list(set(self.other_objects[sec]))
                
            else:
                continue
            '''
            iter_sec = [index for index in range(sec+4, sec+8)]
            other = []
            for future_sec in iter_sec:
                if future_sec in self.other_objects_data:
                    projected_ego_data = self.ego_data[future_sec]
                    
                    # There are multiple ego data at each second
                    for projected_ego_data_per_sec in projected_ego_data:
                        projected_other_car_data = self.other_objects_data[future_sec]
                        for each_car_data in projected_other_car_data:
                            if each_car_data.object_id == 144:
                                #print("ego position_y: original:{} projected:{}_{}, other_car position_y:{}".format(projected_ego_data_per_sec.position_y,projected_ego_data_per_sec.position_y-1.5, projected_ego_data_per_sec.position_y+1.5, each_car_data.position_y))
                                print("ego: ({}, {}) car: ({},{})".format(projected_ego_data_per_sec.position_x, projected_ego_data_per_sec.position_y, each_car_data.position_x,each_car_data.position_y))
                            #if ((projected_ego_data_per_sec.position_y-1.5) > each_car_data.position_y and (projected_ego_data_per_sec.position_y+1.5) < each_car_data.position_y) and ((each_car_data.position_x - projected_ego_data_per_sec.position_x) < 100):
                            #    print("Found")   
        
            
        '''
                                
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
