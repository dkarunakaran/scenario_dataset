import ctypes
import os
import numpy as np
from sys import platform


if platform == "linux" or platform == "linux2":
    se = ctypes.CDLL("/home/beastan/Documents/phd/esmini-bin_ubuntu/bin/libesminiLib.so")
elif platform == "win32":
    se = ctypes.CDLL("./bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

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

def callback(state_ptr, b):
    state = state_ptr.contents
    se.SE_ReportObjectPos(ctypes.c_int(state.id), ctypes.c_float(state.timestamp),
        # position
        ctypes.c_float(state.x - 0.2), ctypes.c_float(state.y + 0.5), ctypes.c_float(0.0),
        # rotation
        ctypes.c_float(state.h + 0.5), ctypes.c_float(state.p), ctypes.c_float(state.r),
        # speed
        ctypes.c_float(state.speed))
    #print("callback for obj {}: speed={}".format(state.id, state.length))

callback_type = ctypes.CFUNCTYPE(None, ctypes.POINTER(SEScenarioObjectState), ctypes.c_void_p)
callback_func = callback_type(callback)

openSCENARIOfile = "/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_files/gate_south-end_north-end_20210608_054523.0_cutin_2.xosc"
se.SE_Init(ctypes.c_char_p(openSCENARIOfile.encode("utf-8")), 0, 0, 0, 0)
obj_state = SEScenarioObjectState()
#se.SE_RegisterObjectCallback(1, callback_func, 0)

maxSimTime = 1023
simTime = 0
dt = 0.1

for i in range(3000):
    for j in range(se.SE_GetNumberOfObjects()):
        se.SE_GetObjectState(j, ctypes.byref(obj_state))
        '''if j==1:
            print('Time {:.2f} esmini speed {:.2f} real speed {:.2f} esmini x: {:.2f} real x:{:.2f} offset:{:.2f}'.format(obj_state.timestamp,obj_state.centerOffsetX,ego_traj[sec]['speed'], obj_state.x,ego_traj[sec]['s'], obj_state.y))

        '''
        if j==1:
            print("speed: {}".format(obj_state.t))
        se.SE_Step()
        if se.SE_GetQuitFlag() == 1:
            break

'''
for _ in np.arange(0, maxSimTime + dt, dt):
    # not the best solution for last dt, can be finally not exact simTime
    # check for current simTime in esmini and comparison with wanted final simTime???
    se.SE_StepDT(ctypes.c_float(dt))
    if se.SE_GetQuitFlag() == 1:
        break
    else:
        simTime += dt
'''
se.SE_Close()
