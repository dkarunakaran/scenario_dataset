import ctypes
import sys
import os
from ctypes import *
import json
from rss import RSS
import math
import numpy as np

rss = RSS()

# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("control", ctypes.c_int),
        ("timestamp", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("r", ctypes.c_float),
        ("roadId", ctypes.c_int),
        ("junctionId", ctypes.c_int),
        ("t", ctypes.c_float),
        ("laneId", ctypes.c_int),
        ("laneOffset", ctypes.c_float),
        ("s", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("height", ctypes.c_float),
        ("objectType", ctypes.c_int),
        ("objectCategory", ctypes.c_int),  
    ]

if sys.platform == "linux" or sys.platform == "linux2":
    se = ctypes.CDLL("/home/beastan/Documents/phd/esmini-bin_ubuntu/bin/libesminiLib.so")
else:
    print("Unsupported platform: {}".format(sys.platform))
    quit()

loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/'
_dir = os.listdir(loc+'scenario_data/')
all_data = []
for _file in _dir:
    print("***************************************************")
    print("started")
    fileDetails = os.path.splitext(_file)
    cutfile = loc+'scenario_data/'+_file
    if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
        with open(cutfile) as f:
            file_data = json.loads(f.read())

        _type = file_data['type']
        count = 1 
        filename = loc+'scenario_files/'+fileDetails[0]+"_"+_type+"_"
        for param in file_data['parameter']: 
            
            #if count == 2 or count == 3:
            #    count += 1
            #    continue

            param_relative_lane_pos = param['param_relative_lane_pos']
            carid = param['param_lane_change_carid']
            
            ego_traj = {}
            other_traj = {}
            last_sec_count = 1
            
            
            manual_trigger = False
            if manual_trigger == True:
                param_relative_lane_pos = []
                with open('manual_data1.json') as f:
                    data = json.loads(f.read())

                param_relative_lane_pos = []
                ego_speed = data['speed']
                ego_d = data['d']
                ego_s = data['s']
                ego_start_to_current_dist=data['start_to_current_dist']
                ego_sec_count = data['sec_count']
                with open('manual_data2.json') as f:
                    data = json.loads(f.read())

                param_relative_lane_pos = []
                other_speed = data['speed']
                other_d = data['d']
                other_s = data['s']
                other_start_to_current_dist=data['start_to_current_dist']
                other_sec_count = data['sec_count']
                for index in range(len(ego_speed)):
                    data = {
                        'ego': {
                            's': ego_s[index],
                            'd': ego_d[index],
                            'speed': ego_speed[index],
                            'start_to_current_dist': ego_start_to_current_dist[index],
                            'sec_count': ego_sec_count[index]
                        },
                        'other': {
                            's': other_s[index],
                            'd': other_d[index],
                            'speed': other_speed[index],
                            'start_to_current_dist':other_start_to_current_dist[index],
                            'sec_count': other_sec_count[index]
                        }
                    }
                    param_relative_lane_pos.append(data)
 
            for item in param_relative_lane_pos:
                #print(item['ego']['sec_count'])
                ego_traj[item['ego']['sec_count']] = {
                    #'lane': item['ego']['lane'], 
                    's': item['ego']['s'], 
                    'd': item['ego']['d'], 
                    'speed': item['ego']['speed'] 
                }
                
                other_traj[item['other']['sec_count']] = {
                    #'lane': item['other']['lane'], 
                    's': item['other']['s'], 
                    'd': item['other']['d'], 
                    'speed': item['other']['speed'] 
                }

                last_sec_count = item['ego']['sec_count']
            
            name = filename+str(count)+".xosc"
            if _type == 'cutin' or _type == 'cutout':
                data = {
                    'file': _file,
                    'type': _type,
                    'carid': carid,
                    'ego_esmini': {
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },
                    'ego_esmini_sec': {
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },

                    'ego_real': {
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },
                    'ego_real_sec': {
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },
                    'adversary_esmini':{
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    }, 
                    'adversary_esmini_sec':{
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    }, 
                    'adversary_real_milli':{
                        'speed': [],
                        'sec': []
                    },
                    'adversary_real':{
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },
                    'adversary_real_sec':{
                        'speed': [],
                        's': [],
                        't': [],
                        'sec': [],
                    },
                    'adversary_esmini_milli':{
                        'speed': [],
                        'sec': []
                    },
                    'esmini':{
                        'rss': [],
                        'sec': []
                    },
                    'real': {
                        'rss': [],
                        'sec': []
                    }

                }
                
                for item in param['param_all_data']['ego']:
                    data['ego_real']['speed'].append(item['speed'])
                    data['ego_real']['s'].append(item['s'])
                    data['ego_real']['t'].append(item['d'])

                #print(param['param_relative_lane_pos'])
                for item in param['param_all_data']['other']:
                    data['adversary_real']['speed'].append(item['speed'])
                    data['adversary_real']['s'].append(item['s'])
                    data['adversary_real']['t'].append(item['d']-6)
                    #data['adversary_real_milli']['speed'].append(item['speed']*3.6)
                
                
                print("Processing the {} file: {}".format(_type, _file))
                se.SE_Init(bytes(name, 'utf-8'), 1, 0, 0, 0)
                obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info
                sec_store = {
                    'ego': [],
                    'adversary': []
                }
                for i in range(2000):
                    rss_enable = False
                    ego_esmini_speed = None
                    ego_real_speed = None
                    adversary_esmini_speed = None
                    adversary_reaparam_relative_lane_posl_speed = None
                    sec_count = 1
                    for j in range(se.SE_GetNumberOfObjects()):
                        se.SE_GetObjectState(j, ctypes.byref(obj_state))
                        sec = int(obj_state.timestamp)+1
                        if sec >= 15:
                            continue
                        if j == 0:
                            if sec not in sec_store['ego']:
                                data['ego_esmini_sec']['speed'].append(obj_state.speed*3.6)
                                data['ego_esmini_sec']['s'].append(obj_state.s)
                                data['ego_esmini_sec']['t'].append(obj_state.t)
                                data['ego_esmini_sec']['sec'].append(sec)
                                if sec <= last_sec_count:
                                    #print(ego_traj[sec]['s'])
                                    data['ego_real_sec']['speed'].append(ego_traj[sec]['speed']*3.6)
                                    data['ego_real_sec']['s'].append(ego_traj[sec]['s'])
                                    data['ego_real_sec']['t'].append(ego_traj[sec]['d'])
                                    data['ego_real_sec']['sec'].append(sec)

                            data['ego_esmini']['speed'].append(obj_state.speed*3.6)
                            data['ego_esmini']['s'].append(obj_state.s)
                            data['ego_esmini']['t'].append(obj_state.t)
                            data['ego_esmini']['sec'].append(sec)
                            sec_store['ego'].append(sec)
                        if j == 1:
                            if sec not in sec_store['adversary']:
                                sec_count = 1
                                data['adversary_esmini_sec']['speed'].append(obj_state.speed*3.6)
                                data['adversary_esmini_sec']['s'].append(obj_state.s)
                                data['adversary_esmini_sec']['t'].append(obj_state.t)
                                data['adversary_esmini_sec']['sec'].append(sec)
                                if sec <= last_sec_count:
                                    data['adversary_real_sec']['speed'].append(other_traj[sec]['speed']*3.6)
                                    data['adversary_real_sec']['s'].append(other_traj[sec]['s'])
                                    data['adversary_real_sec']['t'].append(other_traj[sec]['d']-6)
                                    data['adversary_real_sec']['sec'].append(sec)
                                    data['adversary_esmini_milli']['speed'].append(obj_state.speed*3.6)
                                    data['adversary_esmini_milli']['sec'].append(obj_state.timestamp)

                                    #if(sec < len(param['param_all_data']['other_sec'])):
                                    #    data['adversary_real_milli']['speed'].append(param['param_all_data']['other_sec'][str(sec)][sec_count])
                                    sec_count += 1
                            

                            elif sec in sec_store['adversary']:
                                data['adversary_esmini_milli']['speed'].append(obj_state.speed*3.6)
                                data['adversary_esmini_milli']['sec'].append(obj_state.timestamp)
                                #if(sec < len(param['param_all_data']['other_sec'])):
                                #    data['adversary_real_milli']['speed'].append(param['param_all_data']['other_sec'][str(sec)][sec_count])
                                sec_count += 1
                            data['adversary_esmini']['speed'].append(obj_state.speed*3.6)
                            data['adversary_esmini']['s'].append(obj_state.s)
                            data['adversary_esmini']['t'].append(obj_state.t)
                            data['adversary_esmini']['sec'].append(sec)
                            sec_store['adversary'].append(sec)
                            #print('Time={:.2f} s={:.2f} x={:.2f} t={:.2f}'.format(obj_state.timestamp, obj_state.s, obj_state.x, obj_state.t))
                    se.SE_Step()
                if len(sec_store['ego']) > 2:
                    all_data.append(data)
                count += 1
print("Finished")
print("Then")

exist = False
data_exist = False
if os.path.exists('esmini_real_plot_data.json'):
    exist = True

if exist:
    # Opening JSON file
    f = open('esmini_real_plot_data.json')

    # returns JSON object as
    # a dictionary
    esmini_file_data = json.load(f)

    if 'data' in esmini_file_data:
        for each in all_data:
            esmini_file_data['data'].append(each)
        with open('esmini_real_plot_data.json', 'w') as outfile:
            json.dump(esmini_file_data, outfile)
        data_exist = True

if exist is False or data_exist is False:
    final_data = {
        'data': all_data 
    }

    with open('esmini_real_plot_data.json', 'w') as outfile:
        json.dump(final_data, outfile)

print("Saved")

