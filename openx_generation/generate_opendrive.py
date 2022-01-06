from scenariogeneration import xodr
import os
import json
from collections import OrderedDict

file_loc='/home/beastan/Documents/phd/scenario_extraction/laneletOpenDrive.json'

save_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_files/'

with open(file_loc) as f:
    data = json.loads(f.read())
print("Processing the file: {}".format(file_loc))
filename = os.path.basename(data['file']).split('.', 1)[0]

# create the planview and the geometry
planview = xodr.PlanView()
prev = 0.
for param in data['data']: 
    planview.add_geometry(xodr.Spiral(param['mDiff'], prev, param['length'])) 
    prev = param['mDiff']

rm_solid = xodr.RoadMark(xodr.RoadMarkType.solid,0.2)
centerlane = xodr.Lane(a=2)
centerlane.add_roadmark(rm_solid)

prevLaneList = []
count = 0
laneSecList = []
lanelinker = xodr.LaneLinker()
prevNoLanes = 0 # this has to be replaced with param data
for param in data['data']:
    noOfLanes = param['noLanes']
    
    # create the lanesections
    lanesec = xodr.LaneSection(param['sStart'],centerlane) 
    laneList = []
    for index in range(0, noOfLanes):
        lane = xodr.Lane(a=3)
        lane.add_roadmark(rm_solid)
        lanesec.add_right_lane(lane)
        laneList.append(lane)
    laneSecList.append(lanesec)
    
    # create the lane links
    if count > 0:
        for index in range(0, prevNoLanes):
            lanelinker.add_link(predlane=prevLaneList[index],succlane=laneList[index])

    prevLaneList = laneList
    count += 1
    prevNoLanes = noOfLanes

# create the lanes with the correct links
lanes = xodr.Lanes()
for index in range(0, len(laneSecList)):
    lanes.add_lanesection(laneSecList[index],lanelinker)

# create the road
road = xodr.Road(0,planview,lanes)

# create the opendrive
odr = xodr.OpenDrive('road')
odr.add_road(road)

# adjust the roads and lanes
odr.adjust_roads_and_lanes()
odr.write_xml(save_loc+filename+".xodr")

