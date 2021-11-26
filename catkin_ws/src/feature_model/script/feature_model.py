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

class FeatureModel:
    
    def __init__(self):
        rospy.init_node("FeatureModel node initilization")
        #self.scenario_file = rospy.get_param('~scenario_json_file')
        #self.cars_frenet_file = rospy.get_param('~cars_frenet_json_file')
        #self.ego_frenet_file = rospy.get_param('~ego_frenet_json_file')
        #rospy.Subscriber('/finish_extraction', String, self.process)
        self.scenario_file = "/model/scenario.json"
        self.cars_frenet_file = "/model/cars_frenet.json"
        self.ego_frenet_file = "/model/ego_frenet.json"
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
                start = scenario["start"]
                end = scenario["end"]
                laneChangeCar = scenario["car"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                ego = self.getEgo(start, end)


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
