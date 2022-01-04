from scenariogeneration import xodr
import os

# create the planview and the geometry
planview = xodr.PlanView()
planview.add_geometry(xodr.Line(1000)) # Total length of all lanelets is specified here

# create two different roadmarkings
rm_solid = xodr.RoadMark(xodr.RoadMarkType.solid,0.2)
rm_dashed = xodr.RoadMark(xodr.RoadMarkType.broken,0.2)

# create a centerlane (same centerlane can be used since no linking is needed for this)
centerlane = xodr.Lane(a=2)
centerlane.add_roadmark(rm_solid)

# create the first lanesection with two lanes
lanesec1 = xodr.LaneSection(0,centerlane) # Starting point of each lanelet is specified here
lane1 = xodr.Lane(a=3)
lane1.add_roadmark(rm_solid)

lane2 = xodr.Lane(a=3)
lane2.add_roadmark(rm_solid)

lane3 = xodr.Lane(a=3)
lane3.add_roadmark(rm_solid)

lane4 = xodr.Lane(a=3)
lane4.add_roadmark(rm_solid)

lanesec1.add_right_lane(lane1)
lanesec1.add_right_lane(lane2)
lanesec1.add_right_lane(lane3)
lanesec1.add_right_lane(lane4)

# create the second lanesection with two lanes
lanesec2 = xodr.LaneSection(250,centerlane)
lane5 = xodr.Lane(a=3)
lane5.add_roadmark(rm_solid)

lane6 = xodr.Lane(a=3)
lane6.add_roadmark(rm_solid)

lane7 = xodr.Lane(a=3)
lane7.add_roadmark(rm_solid)

lane8 = xodr.Lane(a=3)
lane8.add_roadmark(rm_solid)

lanesec2.add_right_lane(lane5)
lanesec2.add_right_lane(lane6)
lanesec2.add_right_lane(lane7)
lanesec2.add_right_lane(lane8)

# create the third lanesection with two lanes
lanesec3 = xodr.LaneSection(500,centerlane)
lane9 = xodr.Lane(a=3)
lane9.add_roadmark(rm_solid)

lane10 = xodr.Lane(a=3)
lane10.add_roadmark(rm_solid)

lane11 = xodr.Lane(a=3)
lane11.add_roadmark(rm_solid)

lane12 = xodr.Lane(a=3)
lane12.add_roadmark(rm_solid)

lanesec3.add_right_lane(lane9)
lanesec3.add_right_lane(lane10)
lanesec3.add_right_lane(lane11)
lanesec3.add_right_lane(lane12)

# create the fourth lanesection with two lanes
lanesec4 = xodr.LaneSection(750,centerlane)
lane13 = xodr.Lane(a=3)
lane13.add_roadmark(rm_solid)

lane14 = xodr.Lane(a=3)
lane14.add_roadmark(rm_solid)

lane15 = xodr.Lane(a=3)
lane15.add_roadmark(rm_solid)

lane16 = xodr.Lane(a=3)
lane16.add_roadmark(rm_solid)

lanesec4.add_right_lane(lane13)
lanesec4.add_right_lane(lane14)
lanesec4.add_right_lane(lane15)
lanesec4.add_right_lane(lane16)

# create the lane links
lanelinker = xodr.LaneLinker()
lanelinker.add_link(predlane=lane1,succlane=lane5)
lanelinker.add_link(predlane=lane2,succlane=lane6)
lanelinker.add_link(predlane=lane3,succlane=lane7)
lanelinker.add_link(predlane=lane4,succlane=lane8)
lanelinker.add_link(predlane=lane5,succlane=lane9)
lanelinker.add_link(predlane=lane6,succlane=lane10)
lanelinker.add_link(predlane=lane7,succlane=lane11)
lanelinker.add_link(predlane=lane8,succlane=lane12)
lanelinker.add_link(predlane=lane9,succlane=lane13)
lanelinker.add_link(predlane=lane10,succlane=lane14)
lanelinker.add_link(predlane=lane11,succlane=lane15)
lanelinker.add_link(predlane=lane12,succlane=lane16)

# create the lanes with the correct links
lanes = xodr.Lanes()
lanes.add_lanesection(lanesec1,lanelinker)
lanes.add_lanesection(lanesec2,lanelinker)
lanes.add_lanesection(lanesec3,lanelinker)
lanes.add_lanesection(lanesec4,lanelinker)

# create the road
road = xodr.Road(0,planview,lanes)

# create the opendrive
odr = xodr.OpenDrive('road')
odr.add_road(road)

# adjust the roads and lanes
odr.adjust_roads_and_lanes()

# write the OpenDRIVE file as xodr using current script name
odr.write_xml(os.path.basename(__file__).replace('.py','.xodr'))

