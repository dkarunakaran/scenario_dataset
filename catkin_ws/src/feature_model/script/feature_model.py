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
        self.bag_file = None
        self.parameterCutIn = []
        self.parameterCutOut = []
        self.group_by_sec = OrderedDict()
        self.group_ego_by_sec = OrderedDict()
        rospy.Subscriber('/finish_extraction', String, self.process)
        #self.process("true")
        rospy.on_shutdown(self.shutdown)  
        rospy.spin()
    
    def process(self, msg):
        allScenarios = None;
        with open(self.scenario_file) as json_file1:
            allScenarios = json.load(json_file1)

        self.bag_file = allScenarios['file'] 
        carsData = None
        with open(self.cars_frenet_file) as json_file2:
            carsData = json.load(json_file2)
        egoData = None
        with open(self.ego_frenet_file) as json_file3:
            egoData = json.load(json_file3)
        self.getGroupBySec(carsData);
        self.getGroupEgoBySec(egoData);
        
        
        if "cut-in scenario" in allScenarios.keys():
            cutInScenarios = allScenarios["cut-in scenario"]
            for scenario in cutInScenarios:
                print("_________Cut-in scenario car: {}______________".format(scenario["cutin_car"]))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                laneChangeCar = scenario["cutin_car"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_change_car_data_by_sec = self.getLaneChangeCarBySec(laneChangeCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, "cutin")
            self.saveData(self.parameterCutIn, "cutin");

        if "cut-out scenario" in allScenarios.keys():
            cutOutScenarios = allScenarios["cut-out scenario"]
            for scenario in cutOutScenarios:
                laneChangeCar = scenario["cutout_car"]
                print("_________Cut-out scenario car: {}______________".format(laneChangeCar))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_change_car_data_by_sec = self.getLaneChangeCarBySec(laneChangeCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, "cutout")

            self.saveData(self.parameterCutOut, "cutout")

        '''
        if "lane-following scenario" in allScenarios.keys():
            laneFollowingScenarios = allScenarios["lane-following scenario"]
            for scenario in laneFollowingScenarios:
                laneFollowingCar = scenario["laneFollowing_car"]
                print("_________Cut-out scenario car: {}______________".format(laneChangeCar))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_following_car_data_by_sec = self.getLaneChangeCarBySec(laneFollowingCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneFollowingCar, lane_following_car_data_by_sec, scenario, "lane-following")
        '''
    
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
    
    def getParameters(self, ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, _type):
        if _type == "cutin":
            lane_change_start_time = scenario['cutin_start']
        elif _type == "cutout":
            lane_change_start_time = scenario['cutout_start']
        elif _type == "lane-following":
            lane_change_start_time = scenario['scenario_start']
        count = 0
        found = False
        while True:
            if count > 20:
                break
            if lane_change_start_time in lane_change_car_data_by_sec.keys():
                found = True
                break
            lane_change_start_time += 1
            count +=1
        
        proceed = False
        param_ego_lane_no_init = ego[0][0]['other']['lane_no']
        laneChangeCarData = group_by_car[laneChangeCar]
        param_adv_lane_no_init = laneChangeCarData[0]['other']['lane_no']
        
        if _type == "cutin" and param_ego_lane_no_init != param_adv_lane_no_init:
            proceed = True
        elif _type == "cutout" and param_ego_lane_no_init == param_adv_lane_no_init:
            proceed = True
        elif _type == "lane-following" and param_ego_lane_no_init == param_adv_lane_no_init:
            procced = True

        if found == True and proceed == True:
            if _type == "cutin":
                lane_change_end_time = scenario['cutin_end']
            elif _type == "cutout":
                lane_change_end_time = scenario['cutout_end']
            elif _type == "lane-following":
                lane_change_end_time = scenario['scenario_end']
                if lane_change_end_time in lane_change_car_data_by_sec.keys():
                    while True:
                        if count > 20:
                            break
                        if lane_change_start_time == lane_change_end_time and lane_change_start_time in lane_change_car_data_by_sec.keys():
                            break
                        lane_change_start_time += 1
                        count +=1
            
            if _type == "cutin":
                ego_cutin_start = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['s']
            elif _type == "cutout":
                ego_cutout_start = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['s']
            
            param_ego_speed_init = ego[0][0]['other']['long_speed']
            param_ego_start_pos = ego[0][0]['frenet_data']['s']
            speed = 0
            count = 0
            for sec in self.group_ego_by_sec.keys():
                if sec >=lane_change_start_time and sec <= lane_change_end_time:
                    speed += self.group_ego_by_sec[sec][0]['other']['long_speed']
                    count += 1
            if _type == "cutin":
                param_ego_avg_cutin_speed = speed/count
                print("param_ego_avg_cutin_speed: {}".format(param_ego_avg_cutin_speed))
            elif _type == "cutout":
                param_ego_avg_cutout_speed = speed/count
                print("param_ego_avg_cutout_speed: {}".format(param_ego_avg_cutout_speed))
            
            if _type == "cutin":
                param_ego_to_cutin_dist = ego_cutin_start-param_ego_start_pos
            elif _type == "cutout":
                param_ego_to_cutout_dist = ego_cutout_start-param_ego_start_pos
            
            # Some modification
            if _type == "cutin":
                param_ego_to_cutin_dist = param_ego_to_cutin_dist/param_ego_avg_cutin_speed
                print("param_ego_to_cutin_dist: {}".format(param_ego_to_cutin_dist))
            elif _type == "cutout":
                param_ego_to_cutout_dist = param_ego_to_cutout_dist/param_ego_avg_cutout_speed
                print("param_ego_to_cutout_dist: {}".format(param_ego_to_cutout_dist))

            if _type == "cutin":
                param_ego_to_cutin_time = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['sec'] - ego[0][0]['frenet_data']['sec'] 
                print("param_ego_to_cutin_time: {}".format(param_ego_to_cutin_time))
            elif _type == "cutout":
                param_ego_to_cutout_time = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['sec'] - ego[0][0]['frenet_data']['sec'] 
                print("param_ego_to_cutout_time: {}".format(param_ego_to_cutout_time))

            if param_ego_speed_init < 0.5:
                speed = 0
                count = 0
                for sec in self.group_ego_by_sec.keys():
                    speed += self.group_ego_by_sec[sec][0]['other']['long_speed']
                    count += 1
                param_ego_speed_init = speed/count

            print("Ego initial speed: {}, lane number init: {}".format(param_ego_speed_init, param_ego_lane_no_init))
            
            end = scenario["scenario_end"]
            while end not in self.group_ego_by_sec.keys():
                end -= 1

            param_ego_speed_final = self.group_ego_by_sec[end][0]['other']['long_speed']
            print("param_ego_speed_final: {}".format(param_ego_speed_final))
            
            param_adv_speed_init = param_ego_speed_init
            
            param_start_diff=(laneChangeCarData[0]['frenet_data']['s']-ego[0][0]['frenet_data']['s'])
            param_adv_start_pos = laneChangeCarData[0]['frenet_data']['s']
            print("Start diff: {}, adversary start pos: {}, ego start pos: {}".format(param_start_diff, param_adv_start_pos, param_ego_start_pos))
           
            # Find cut-in time
            found = False
            param_cut_time = lane_change_end_time-lane_change_start_time
            param_adv_lane_no_final = param_adv_lane_no_init
            cutin_start_sec = lane_change_start_time
            cutin_end_sec = None
            for sec in lane_change_car_data_by_sec.keys():
                if sec in lane_change_car_data_by_sec.keys():
                    size = len(lane_change_car_data_by_sec[sec])-1
                    lane_no = lane_change_car_data_by_sec[sec][size]['other']['lane_no']
                    if lane_no != param_adv_lane_no_init:
                        param_adv_lane_no_final = lane_change_car_data_by_sec[sec][size]['other']['lane_no']
                        cutin_end_sec = sec
                        found = True
                        break
            if found == True:
                param_cut_time = (cutin_end_sec-cutin_start_sec)-1
            
            param_cut_time = 3

            if _type == "cutin":
                param_cutin_time = param_cut_time
                print("param_cutin_time: {}".format(param_cutin_time))
            elif _type == "cutout":
                param_cutout_time = param_cut_time
                print("param_cutout_time: {}".format(param_cutout_time))
                print("param_adv_lane_no_final: {}".format(param_adv_lane_no_final))

            startDiffProceed = True
            # Find cut-in distance:
            adv_cut_start = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['s']
            if _type == "cutin":
                param_cutin_start_dist = adv_cut_start - ego_cutin_start
                print("init_diff: {} and param_cutin_start_dist: {}".format(param_start_diff, param_cutin_start_dist))
                start_diff = abs(param_start_diff-param_cutin_start_dist)
                if start_diff > 45:
                    startDiffProceed = False
            elif _type == "cutout":
                param_cutout_start_dist = adv_cut_start - ego_cutout_start
                print("init_diff: {} and param_cutout_start_dist: {}".format(param_start_diff, param_cutout_start_dist))
                


            if _type == "cutin":
                if param_cutin_start_dist > param_start_diff:
                    param_adv_speed_init = param_ego_speed_init+2
                print("Adversary initial speed: {}, lane number init: {}".format(param_adv_speed_init, param_adv_lane_no_init))
            elif _type == "cutout":
                if param_cutout_start_dist > param_start_diff:
                    param_adv_speed_init = param_ego_speed_init+2
                print("Adversary initial speed: {}, lane number init: {}".format(param_adv_speed_init, param_adv_lane_no_init))
                
            # Find average cut-in speed
            speed = 0
            count = 1
            for sec in lane_change_car_data_by_sec.keys():
                if lane_change_car_data_by_sec[sec][0]['other']['long_speed'] < 0:
                    continue
                if sec >=lane_change_start_time and sec <= lane_change_end_time:
                    speed += lane_change_car_data_by_sec[sec][0]['other']['long_speed']
                    count += 1
            
            if _type == "cutin":
                param_adv_avg_cutin_speed = speed/count
                if param_adv_avg_cutin_speed == 0.0:
                    param_adv_avg_cutin_speed = 0.05
                print("param_adv_avg_cutin_speed: {}".format(param_adv_avg_cutin_speed))
            elif _type == "cutout":
                param_adv_avg_cutout_speed = speed/count
                if param_adv_avg_cutout_speed == 0.0:
                    param_adv_avg_cutout_speed = 0.05
                print("param_adv_avg_cutout_speed: {}".format(param_adv_avg_cutout_speed))
            

            if _type == "cutin":
                param_adv_to_cutin_dist = adv_cut_start-param_adv_start_pos
                # Some modification
                param_adv_to_cutin_dist = param_adv_to_cutin_dist/param_adv_avg_cutin_speed
                print("param_adv_to_cutin_dist: {}".format(param_adv_to_cutin_dist))

            elif _type == "cutout":
                param_adv_to_cutout_dist = adv_cut_start-param_adv_start_pos
                # Some modification
                param_adv_to_cutout_dist = param_adv_to_cutout_dist/param_adv_avg_cutout_speed
                print("param_adv_to_cutout_dist: {}".format(param_adv_to_cutout_dist))
            
            if _type == "cutin":
                param_adv_to_cutin_time = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec'] - laneChangeCarData[0]['frenet_data']['sec'] 
                print("param_adv_to_cutin_time: {}".format(param_adv_to_cutin_time))
            elif _type == "cutout":
                param_adv_to_cutout_time = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec'] - laneChangeCarData[0]['frenet_data']['sec'] 
                print("param_adv_to_cutout_time: {}".format(param_adv_to_cutout_time))
            
            end = scenario["scenario_end"]
            while end not in lane_change_car_data_by_sec.keys():
                end -= 1

            param_adv_speed_final = lane_change_car_data_by_sec[end][0]['other']['long_speed']
            print("param_adv_speed_final: {}".format(param_adv_speed_final))
           
            if _type == "cutin" and startDiffProceed == True:
                # some modification
                if param_cutin_time == 0:
                    param_cutin_time = 4

                trigger_cond = 0
                if param_start_diff > param_cutin_start_dist:
                    trigger_cond = 0
                elif param_start_diff == param_cutin_start_dist:
                    trigger_cond = 0
                    param_cutin_start_dist -= 5
                    param_ego_speed_init += 0.5
                else:
                    trigger_cond = 1
                    ratio = param_cutin_start_dist/param_start_diff
                    param_adv_speed_init += ratio
           
                if param_cutin_start_dist >= 0: 
                    param = {
                            'param_ego_speed_init': param_ego_speed_init,
                            'param_ego_lane_no_init': param_ego_lane_no_init,
                            'param_ego_start_pos': param_ego_start_pos,
                            'param_adv_speed_init': param_adv_speed_init,
                            'param_adv_lane_no_init': param_adv_lane_no_init,
                            'param_start_diff': param_start_diff,
                            'param_adv_start_pos': param_adv_start_pos,
                            'param_cutin_start_dist': param_cutin_start_dist,
                            'param_cutin_time': param_cutin_time,
                            'param_adv_avg_cutin_speed': param_adv_avg_cutin_speed,
                            'param_ego_avg_cutin_speed': param_ego_avg_cutin_speed,
                            'param_adv_to_cutin_dist': param_adv_to_cutin_dist,
                            'param_adv_to_cutin_time': param_adv_to_cutin_time,
                            'param_ego_to_cutin_dist': param_ego_to_cutin_dist,
                            'param_ego_to_cutin_time': param_ego_to_cutin_time,
                            'param_trigger_cond': trigger_cond
                    }
                    self.parameterCutIn.append(param)
                    print("Saving cut-in data")
                else:
                    print("Not saved cut-in data")
            elif _type == "cutout":
                # some modification
                if param_cutout_time == 0:
                    param_cutout_time = 4

                trigger_cond = 0
                if param_start_diff > param_cutout_start_dist:
                    trigger_cond = 0
                elif param_start_diff == param_cutout_start_dist:
                    trigger_cond = 0
                    param_cutout_start_dist -= 5
                    param_ego_speed_init += 0.5
                else:
                    trigger_cond = 1
                    ratio = param_cutout_start_dist/param_start_diff
                    param_adv_speed_init += ratio

                param = {
                        'param_ego_speed_init': param_ego_speed_init,
                        'param_ego_lane_no_init': param_ego_lane_no_init,
                        'param_ego_start_pos': param_ego_start_pos,
                        'param_adv_speed_init': param_adv_speed_init,
                        'param_adv_lane_no_init': param_adv_lane_no_init,
                        'param_start_diff': param_start_diff,
                        'param_adv_start_pos': param_adv_start_pos,
                        'param_cutout_start_dist': param_cutout_start_dist,
                        'param_cutout_time': param_cutout_time,
                        'param_adv_avg_cutout_speed': param_adv_avg_cutout_speed,
                        'param_ego_avg_cutout_speed': param_ego_avg_cutout_speed,
                        'param_adv_to_cutout_dist': param_adv_to_cutout_dist,
                        'param_adv_to_cutout_time': param_adv_to_cutout_time,
                        'param_ego_to_cutout_dist': param_ego_to_cutout_dist,
                        'param_ego_to_cutout_time': param_ego_to_cutout_time,
                        'param_trigger_cond': trigger_cond,
                        'param_adv_lane_no_final': param_adv_lane_no_final
                }
                self.parameterCutOut.append(param)
                print("Saving cut-out data")
            else:
                print("Not saved the data")

        else:
            print("Not saved the data")

    def saveData(self, parameter, _type="cutin"):
        data = {
            'type': _type,
            'parameter': parameter 
        }
        base = os.path.basename(self.bag_file)
        filename = os.path.splitext(base)[0]
        with open(self.parameter_dir+filename+"."+_type, 'w') as outfile:
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
