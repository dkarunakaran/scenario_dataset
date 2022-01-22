import ctypes
import sys
import os
from ctypes import *
import json

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
_dir = os.listdir(loc)
for _file in _dir:
    base = os.path.basename(_file)
    fileDetails = os.path.splitext(base)
    if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
        with open(_file) as f:
            file_data = json.loads(f.read())

        _type = file_data['type']
        count = 1 
        for param in file_data['parameter']: 
            param_relative_lane_pos = param['param_relative_lane_pos']
            ego_traj = {}
            other_traj = {}
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


            name = loc+'scenario_files/'+fileDetails[0]+"_"+_type+"_"+str(count)+".xosc"
            if _type == 'cutin' or _type == 'cutout':
                data = {
                    'file': _file,
                    'type': _type,
                    'ego_esmini': {
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': []
                    },
                    'ego_real': {
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': []
                    },
                    'adversary_esmini':{
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': []
                    }, 
                    'adversary_real':{
                        'speed': [],
                        'x': [],
                        'y': [],
                        'sec': []
                    }
                }
                print("Processing the {} file: {}".format(_type, _file))
                se.SE_Init(bytes(name, 'utf-8'), 1, 0, 0, 0)
                obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info

                sec_store = {
                    'ego': [],
                    'adversary': []
                }

                for i in range(3000):
                    for j in range(se.SE_GetNumberOfObjects()):
                        se.SE_GetObjectState(j, ctypes.byref(obj_state))
                        sec = int(obj_state.timestamp)
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
                                sec_store['ego'].append(sec)
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
                                sec_store['adversary'].append(sec)
                        
                        print('Time {:.2f} ObjId {} speed {:.2f} x {:.2f}'.format(obj_state.timestamp, j, obj_state.centerOffsetX, obj_state.x))
                    se.SE_Step()
               
                count += 1
