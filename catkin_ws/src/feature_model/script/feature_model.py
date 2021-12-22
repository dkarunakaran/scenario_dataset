#!/usr/bin/env python

from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import json
import tf
import math
import rosbag
from std_msgs.msg import String
from collections import OrderedDict
import os

class FeatureModel:
    
    def __init__(self):
        rospy.init_node("FeatureModel node initilization")
        self.scenario_file = rospy.get_param('~scenario_json_file')
        self.cars_frenet_file = rospy.get_param('~cars_frenet_json_file')
        self.ego_frenet_file = rospy.get_param('~ego_frenet_json_file')
        self.parameter_dir = rospy.get_param('~parameter_json_dir')
        self.bag_file = rospy.get_param('~bag_file')
        #rospy.Subscriber('/finish_extraction', String, self.process)
        #self.scenario_file = "/model/scenario.json"
        #self.cars_frenet_file = "/model/cars_frenet.json"
        #self.ego_frenet_file = "/model/ego_frenet.json"
        self.parameter = [] 
        self.group_by_sec = OrderedDict()
        self.group_ego_by_sec = OrderedDict()
        self.process("true")
        rospy.on_shutdown(self.shutdown)   
    
    def process(self, msg):
        allScenarios = None;
        with open(self.scenario_file) as json_file1:
            allScenarios = json.load(json_file1)
        if "cut-in scenario" in allScenarios.keys():
            carsData = None
            with open(self.cars_frenet_file) as json_file2:
                carsData = json.load(json_file2)
            egoData = None
            with open(self.ego_frenet_file) as json_file3:
                egoData = json.load(json_file3)
            self.getGroupBySec(carsData);
            self.getGroupEgoBySec(egoData);
            cutInScenarios = allScenarios["cut-in scenario"]
            for scenario in cutInScenarios:
                start = scenario["cutin_start"]
                end = scenario["cutin_end"]
                laneChangeCar = scenario["cutin_car"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                ego = self.getEgo(start, end)
                lane_change_car_data_by_sec = self.getLaneChangeCarBySec(laneChangeCar, group_by_car)
                self.getParameters(ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario)
            self.saveData();
    
    def getLaneChangeCarBySec(self, laneChangeCar, group_by_car):
        laneChangeCarData = group_by_car[laneChangeCar]
        laneChangeCarData_by_sec = OrderedDict()
        for data in laneChangeCarData:
            if data['frenet_data']['sec'] in laneChangeCarData_by_sec:
                laneChangeCarData_by_sec[data['frenet_data']['sec']].append(data)
            else:
                laneChangeCarData_by_sec[data['frenet_data']['sec']] = []
                laneChangeCarData_by_sec[data['frenet_data']['sec']].append(data)
        
        return laneChangeCarData_by_sec
    
    def getParameters(self, ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario):
        lane_change_start_time = scenario['cutin_start']
        lane_change_end_time = scenario['cutin_end']
        param_ego_speed_init = ego[0][0]['other']['long_speed']
        param_ego_lane_no_init = ego[0][0]['other']['lane_no']
        param_ego_start_pos = ego[0][0]['frenet_data']['s']
        print("Ego initial speed: {}, lane number init: {}".format(param_ego_speed_init, param_ego_lane_no_init))
        param_ego_speed_interval = {}
        param_ego_speed_interval[0] = param_ego_speed_init
        current_dist = param_ego_start_pos;
        '''index = 0
        s = 0
        for each in ego:
            dist = each[0]['frenet_data']['s']
            dist_diff = dist -current_dist;
            if dist_diff >= 1:
                s += dist_diff
                current_dist = dist
                param_ego_speed_interval[int(s)] = ego[index][0]['other']['long_speed']
            index += 1
        '''
        s = 0
        for sec in self.group_ego_by_sec.keys():
            dist = self.group_ego_by_sec[sec][0]['frenet_data']['s']
            dist_diff = dist-current_dist;
            s += dist_diff
            current_dist = dist
            param_ego_speed_interval[int(s)] = self.group_ego_by_sec[sec][0]['other']['long_speed']

        print(param_ego_speed_interval)

        laneChangeCarData = group_by_car[laneChangeCar]
        param_adv_speed_init = laneChangeCarData[0]['other']['long_speed']
        param_adv_lane_no_init = laneChangeCarData[0]['other']['lane_no']
        print("Adversary initial speed: {}, lane number init: {}".format(param_adv_speed_init, param_adv_lane_no_init))
        param_start_diff = (laneChangeCarData[0]['frenet_data']['s']-ego[0][0]['frenet_data']['s'])+2
        param_adv_start_pos = ego[0][0]['frenet_data']['s']+param_start_diff
        print("Start diff: {}, adversary start pos: {}, ego start pos: {}".format(param_start_diff, param_adv_start_pos, param_ego_start_pos))
        param_adv_speed_interval = {}
        param_adv_speed_interval[0] = param_adv_speed_init
        current_dist = param_adv_start_pos;
        s = 0
        for sec in lane_change_car_data_by_sec.keys():
            dist = lane_change_car_data_by_sec[sec][0]['frenet_data']['s']
            dist_diff = dist-current_dist;
            s += dist_diff
            current_dist = dist
            param_adv_speed_interval[int(s)] = lane_change_car_data_by_sec[sec][0]['other']['long_speed']
        print(param_adv_speed_interval) 
        
        param_adv_speed_interval_time = {}
        '''
        param_adv_speed_interval_time[0] = laneChangeCarData[0]['other']['long_speed']
        t = 0 
        index = 0
        current_sec = laneChangeCarData[0]['frenet_data']['sec']
        for sec in lane_change_car_data_by_sec.keys():
            print(sec)
            sec_diff = sec - current_sec
            if sec_diff >= 1.0:
                t += sec_diff
                current_sec = sec
                param_adv_speed_interval_time[t] = lane_change_car_data_by_sec[sec][0]['other']['long_speed']

        print(param_adv_speed_interval_time)
        '''

        found = False
        param_cutin_time = lane_change_end_time-lane_change_start_time
        cutin_start_sec = None
        for sec in lane_change_car_data_by_sec.keys():
            next_lane_no = lane_change_car_data_by_sec[sec+1][0]['other']['lane_no']
            if next_lane_no != param_adv_lane_no_init:
                cutin_start_sec = sec
                found = True
                break
        
        if found == True:
            param_cutin_time = (lane_change_end_time-cutin_start_sec)+1

        # Find cut-in distance:
        adv_start = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['s']
        ego_start = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['s']
        param_cutin_start_dist = adv_start - ego_start
        print("init_diff: {} and param_cutin_start_dist: {}".format(param_start_diff, param_cutin_start_dist))
        param_ego_dist_travelled = ego_start - param_ego_start_pos
        param = {
                'param_ego_speed_init': param_ego_speed_init,
                'param_ego_lane_no_init': param_ego_lane_no_init,
                'param_ego_start_pos': param_ego_start_pos,
                'param_ego_speed_interval': param_ego_speed_interval,
                'param_adv_speed_init': param_adv_speed_init,
                'param_adv_lane_no_init': param_adv_lane_no_init,
                'param_start_diff': param_start_diff,
                'param_adv_start_pos': param_adv_start_pos,
                'param_adv_speed_interval': param_adv_speed_interval,
                'param_adv_speed_interval_time': param_adv_speed_interval_time,
                'param_cutin_start_dist': param_cutin_start_dist,
                'param_ego_dist_travelled': param_ego_dist_travelled,
                'param_cutin_time': param_cutin_time
        }
        self.parameter.append(param)
        

    def saveData(self):
        data = {
            'parameter': self.parameter 
        }
        filename = os.path.basename(self.bag_file).split('.', 1)[0]
        with open(self.parameter_dir+filename+".json", 'w') as outfile:
            json.dump(data, outfile)

    def getGroupBySec(self, carsData):
        for eachData in carsData:
            for data in eachData: 
                if data["frenet_data"]["sec"] in self.group_by_sec:
                    self.group_by_sec[data["frenet_data"]["sec"]].append(data);
                else:
                    self.group_by_sec[data["frenet_data"]["sec"]] = []
                    self.group_by_sec[data["frenet_data"]["sec"]].append(data)

    def getGroupEgoBySec(self, egoData):
        for data in egoData:
            if data["frenet_data"]["sec"] in self.group_ego_by_sec:
                self.group_ego_by_sec[data["frenet_data"]["sec"]].append(data);
            else:
                self.group_ego_by_sec[data["frenet_data"]["sec"]] = []
                self.group_ego_by_sec[data["frenet_data"]["sec"]].append(data)

    def getEgo(self, start, end):
        data = []
        for sec in self.group_ego_by_sec:
            if sec >= start and sec <= end:
                data.append(self.group_ego_by_sec[sec]);
        
        return data


    def getAllCars(self, start = None, end = None, laneChangeCar = None):
        data = []
        for sec in self.group_by_sec:
            if sec >= start and sec <= end:
                data.append(self.group_by_sec[sec]);
        
        return data

    def groupByCar(self, listData):
        group_by_car = OrderedDict()
        for eachData in listData:
            for data in eachData:
                if data["car_id"] in group_by_car:
                    group_by_car[data["car_id"]].append(data);
                else:
                    group_by_car[data["car_id"]] = [];
                    group_by_car[data["car_id"]].append(data);
        
        return group_by_car
            
    def shutdown(self):
        print("FeatureModel node is shutting down!!!")

if __name__ == '__main__':
    try:
        FeatureModel()
    except rospy.ROSInterruptException:
	rospy.logerr('Could not start FeatureModel node.')
