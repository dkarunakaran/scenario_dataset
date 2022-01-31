import ctypes
import sys
import os
from ctypes import *
import json
from rss import RSS

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
            param_relative_lane_pos = param['param_relative_lane_pos']
            carid = param['param_lane_change_carid']
            ego_traj = {}
            other_traj = {}
            last_sec_count = 1
            for item in param_relative_lane_pos:
                ego_traj[item['ego']['sec_count']] = {
                    'lane': item['ego']['lane'], 
                    's': item['ego']['s'], 
                    'd': item['ego']['d'], 
                    'speed': item['ego']['speed'] 
                }
                
                other_traj[item['other']['sec_count']] = {
                    'lane': item['other']['lane'], 
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
                        'x': [],
                        'y': [],
                        'sec': [],
                    },
                    'ego_real': {
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': [],
                    },
                    'adversary_esmini':{
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': [],
                    }, 
                    'adversary_real':{
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': [],
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
                print("Processing the {} file: {}".format(_type, _file))
                se.SE_Init(bytes(name, 'utf-8'), 1, 1, 0, 0)
                obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info
                sec_store = {
                    'ego': [],
                    'adversary': []
                }
                for i in range(3000):
                    rss_enable = False
                    ego_esmini_speed = None
                    ego_real_speed = None
                    adversary_esmini_speed = None
                    adversary_real_speed = None
                    for j in range(se.SE_GetNumberOfObjects()):
                        se.SE_GetObjectState(j, ctypes.byref(obj_state))
                        sec = int(obj_state.timestamp)+1
                        if sec > last_sec_count:
                            continue
                        if j == 0:
                            if sec in sec_store['ego']:
                                continue
                            else:
                                data['ego_esmini']['speed'].append(obj_state.centerOffsetX)
                                data['ego_esmini']['x'].append(obj_state.x)
                                data['ego_esmini']['y'].append(obj_state.y)
                                data['ego_esmini']['sec'].append(sec)
                                data['ego_real']['speed'].append(ego_traj[sec]['speed'])
                                data['ego_real']['x'].append(ego_traj[sec]['s'])
                                data['ego_real']['y'].append(ego_traj[sec]['d'])
                                data['ego_real']['sec'].append(sec)
                                rss_enable = True
                                ego_esmini_speed = obj_state.centerOffsetX 
                                ego_real_speed = ego_traj[sec]['speed']
                                sec_store['ego'].append(sec)
                                print('Time {:.2f} esmini speed {:.2f} real speed {:.2f} esmini x: {:.2f} real x:{:.2f} offset:{:.2f}'.format(obj_state.timestamp,obj_state.centerOffsetX,ego_traj[sec]['speed'], obj_state.x,ego_traj[sec]['s'], obj_state.y))
                        if j == 1:
                            if sec in sec_store['adversary']:
                               continue
                            else:
                                data['adversary_esmini']['speed'].append(obj_state.centerOffsetX)
                                data['adversary_esmini']['x'].append(obj_state.x)
                                data['adversary_esmini']['y'].append(obj_state.y)
                                data['adversary_esmini']['sec'].append(sec)
                                data['adversary_real']['speed'].append(other_traj[sec]['speed'])
                                data['adversary_real']['x'].append(other_traj[sec]['s'])
                                data['adversary_real']['y'].append(other_traj[sec]['d'])
                                data['adversary_real']['sec'].append(sec)
                                rss_enable = True
                                adversary_esmini_speed = obj_state.centerOffsetX 
                                adversary_real_speed = other_traj[sec]['speed'] 
                                sec_store['adversary'].append(sec)
                        
                    if rss_enable:
                        rss_esmini_dist = rss.calculate_rss_safe_dist(ego_esmini_speed, adversary_esmini_speed)
                        rss_real_dist = rss.calculate_rss_safe_dist(ego_real_speed, adversary_real_speed)
                        data['esmini']['rss'].append(rss_esmini_dist);
                        data['esmini']['sec'] = sec_store['ego']
                        data['real']['rss'].append(rss_real_dist);
                        data['real']['sec'] = sec_store['ego']
                        print("rss_esmini_dist: {}, rss_real_dist:{}".format(rss_esmini_dist, rss_real_dist))
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
