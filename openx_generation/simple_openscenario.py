""" 
    An example showing how to create a trajector based on polyline 
    Also shows how to create a vehicle from start
    Some features used:
    
    - RelativeLanePosition 
    
    - Polyline
    
    - Trajectory
    
    - TimeHeadwayCondition
    
    - FollowTrajectoryAction
        
"""
import os
from scenariogeneration import xosc, prettyprint

### create catalogs
catalog = xosc.Catalog()



### create road
road = xosc.RoadNetwork(roadfile='gate_south-end_north-end_20210608_054523.0.xodr')


### create parameters
paramdec = xosc.ParameterDeclarations()

paramdec.add_parameter(xosc.Parameter('$HostVehicle',xosc.ParameterType.string,'car_white'))
paramdec.add_parameter(xosc.Parameter('$TargetVehicle',xosc.ParameterType.string,'car_red'))


bb = xosc.BoundingBox(2,5,1.8,2.0,0,0.9)
fa = xosc.Axle(0.523598775598,0.8,1.68,2.98,0.4)
ba = xosc.Axle(0.523598775598,0.8,1.68,0,0.4)
white_veh = xosc.Vehicle('car_white',xosc.VehicleCategory.car,bb,fa,ba,69,10,10)

white_veh.add_property_file('../models/car_white.osgb')
white_veh.add_property('model_id','0')


bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0,0.8)
fa = xosc.Axle(0.523598775598,0.8,1.68,2.98,0.4)
ba = xosc.Axle(0.523598775598,0.8,1.68,0,0.4)
red_veh = xosc.Vehicle('car_red',xosc.VehicleCategory.car,bb,fa,ba,69,10,10)

red_veh.add_property_file('../models/car_red.osgb')
red_veh.add_property('model_id','2')

## create entities

egoname = 'Ego'
targetname = 'Target'

entities = xosc.Entities()
entities.add_scenario_object(egoname,white_veh)
entities.add_scenario_object(targetname,red_veh)


### create init

init = xosc.Init()
step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,1)

egospeed = xosc.AbsoluteSpeedAction(30,step_time)
egostart = xosc.TeleportAction(xosc.LanePosition(40,0,-1,"0"))

targetspeed = xosc.AbsoluteSpeedAction(40,step_time)
targetstart = xosc.TeleportAction(xosc.LanePosition(30,0,-2,"0"))

init.add_init_action(egoname,egospeed)
init.add_init_action(egoname,egostart)
init.add_init_action(targetname,targetspeed)
init.add_init_action(targetname,targetstart)


### create an event

trigcond = xosc.TimeHeadwayCondition(targetname,0.4,xosc.Rule.greaterThan)

trigger = xosc.EntityTrigger('mytesttrigger',0.2,xosc.ConditionEdge.rising,trigcond,egoname)

event = xosc.Event('myfirstevent',xosc.Priority.overwrite)
event.add_trigger(trigger)

positionlist = []
positionlist.append(xosc.LanePosition(0,0,-2,"0"))
positionlist.append(xosc.LanePosition(20,0,-2,"0"))
positionlist.append(xosc.LanePosition(40,0,-2,"0"))
positionlist.append(xosc.LanePosition(60,0,-2,"0"))

polyline = xosc.Polyline([0,0.5,1,1.5],positionlist)


traj = xosc.Trajectory('my_trajectory',False)
traj.add_shape(polyline)



trajact = xosc.FollowTrajectoryAction(traj,xosc.FollowMode.position,xosc.ReferenceContext.relative,1,0)

event.add_action('newspeed',trajact)


## create the act
man = xosc.Maneuver('my_maneuver')
man.add_event(event)

mangr = xosc.ManeuverGroup('mangroup')
mangr.add_actor('$owner')
mangr.add_maneuver(man)
starttrigger = xosc.ValueTrigger('starttrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
act = xosc.Act('my_act',starttrigger)
act.add_maneuver_group(mangr)

## create the story
storyparam = xosc.ParameterDeclarations()
storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,targetname))
story = xosc.Story('mystory',storyparam)
story.add_act(act)


## create the storyboard
sb = xosc.StoryBoard(init)
sb.add_story(story)

## create the scenario
sce = xosc.Scenario('trajectory_example','Mandolin',paramdec,entities=entities,storyboard = sb,roadnetwork=road,catalog=catalog)

# Print the resulting xml
prettyprint(sce.get_element())

# write the OpenSCENARIO file as xosc using current script name
sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))

# uncomment the following lines to display the scenario using esmini
# from scenariogeneration import esmini
# esmini(sce,os.path.join('esmini'))
