import os
from scenariogeneration import xosc, prettyprint

### create catalogs
catalog = xosc.Catalog()
catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

road = xosc.RoadNetwork(roadfile='road1.xodr')

### create parameters
ego_vehicle = 'hero'
ego_velocity_init = 20 # param 0
ego_start_pos = 20 # param 1 - s coordinate of road frame
ego_start_lane = 1 # param 2 - lane number
adversary_vehicle = 'adversary'
adversary_velocity_init = 30 # param 3
# Difference between ego vehicle and adversary, positive value indicates the
# Target location is ahead of ego vehicle
adversary_pos = 1 # param 4 - front=0 or back=1
adversary_start_diff = 10 # param 5  - difference in s coordinate of road frame
if adversary_pos == 1:
    adversary_start_diff *= -1
adversary_start_pos = ego_start_pos+adversary_start_diff
adversary_lane = 2 # param 6 - lane number
adversary_cutin_speed = 40 # param 7 
adversary_cutin_trigger_dist = 5 # param 8 - distance in s from target to ego
adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2

adversary_cutin_time = 3 # Param 10 time taken to complete the lane change

paramdec = xosc.ParameterDeclarations()
paramdec.add_parameter(xosc.Parameter('$egoVehicle',xosc.ParameterType.string, ego_vehicle))
paramdec.add_parameter(xosc.Parameter('$egoVelocityInit',xosc.ParameterType.double, ego_velocity_init))
paramdec.add_parameter(xosc.Parameter('$egoStartPos',xosc.ParameterType.double, ego_start_pos))
paramdec.add_parameter(xosc.Parameter('$egoStartLane',xosc.ParameterType.double, ego_start_lane))
paramdec.add_parameter(xosc.Parameter('$adversaryVehicle',xosc.ParameterType.string, adversary_vehicle))
paramdec.add_parameter(xosc.Parameter('$adversaryVelocityInit',xosc.ParameterType.double, adversary_velocity_init))
paramdec.add_parameter(xosc.Parameter('$adversaryPos',xosc.ParameterType.integer, adversary_pos))
paramdec.add_parameter(xosc.Parameter('$adversaryLane',xosc.ParameterType.double,adversary_lane))
paramdec.add_parameter(xosc.Parameter('$adversaryCutinSpeed',xosc.ParameterType.double,adversary_cutin_speed))
paramdec.add_parameter(xosc.Parameter('$adversaryCutinTriggerDist',xosc.ParameterType.double,adversary_cutin_trigger_dist))
paramdec.add_parameter(xosc.Parameter('$adversaryCutinTime',xosc.ParameterType.double,adversary_cutin_time))

# vehicle.tesla.model3
bb = xosc.BoundingBox(2.1,4.5,1.8,1.5,0,0.9)
fa = xosc.Axle(0.5,0.6,1.8,3.1,0.3)
ba = xosc.Axle(0.0,0.6,1.8,0.0,0.3)
ego_veh = xosc.Vehicle('$egoVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
ego_veh.add_property('model_id','0')
ego_veh.add_property('type','ego_vehicle')
bb = None
fa = None
ba = None
if adversary_vehicle_model == 0:
    bb = xosc.BoundingBox(2.0,5.0,1.8,1.4,0.0,0.9)
    fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
    ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
elif adversary_vehicle_model == 1:
    bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0.0,0.8)
    fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
    ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
elif adversary_vehicle_model == 2:
    bb = xosc.BoundingBox(0.9,2.2,1.3,0.40,0.0,0.65)
    fa = xosc.Axle(1.5,0.7,0.1,1.5,0.35)
    ba = xosc.Axle(0.0,0.7,0.1,0.0,0.35)

adversary_veh = xosc.Vehicle('$adversaryVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
adversary_veh.add_property('model_id','1')

prop = xosc.Properties()
prop.add_property(name="module", value="external_control")
cont = xosc.Controller('carlaControler',prop)

entities = xosc.Entities()
entities.add_scenario_object('$egoVehicle',ego_veh, controller=cont)
entities.add_scenario_object('$adversaryVehicle',adversary_veh)

### create init
init = xosc.Init()
step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,1)
egospeed = xosc.AbsoluteSpeedAction('$egoVelocityInit',step_time)
egostart = xosc.TeleportAction(xosc.LanePosition('$egoStartPos',0, '$egoStartLane',"0"))

targetspeed = xosc.AbsoluteSpeedAction('$adversaryVelocityInit',step_time)
targetstart = xosc.TeleportAction(xosc.LanePosition('$adversaryPos',0,'$adversaryLane',"0"))

init.add_init_action('$egoVehicle',egospeed)
init.add_init_action('$egoVehicle',egostart)
init.add_init_action('$adversaryVehicle',targetspeed)
init.add_init_action('$adversaryVehicle',targetstart)

### create an event
trig_cond1 = xosc.RelativeDistanceCondition('$adversaryCutinTriggerDist',xosc.Rule.greaterThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
trigger = xosc.EntityTrigger('cutinTrigger',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
event = xosc.Event('cutInevent',xosc.Priority.overwrite)
event.add_trigger(trigger)

sin_time= xosc.TransitionDynamics(xosc.DynamicsShapes.sinusoidal,xosc.DynamicsDimension.time,'$adversaryCutinTime')
action = xosc.AbsoluteLaneChangeAction('$egoStartLane',sin_time)
event.add_action('cutInLaneAction',action)

lin_time= xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time,'$adversaryCutinTime')
action = xosc.AbsoluteSpeedAction('$adversaryCutinSpeed',lin_time)
event.add_action('cutInSpeedAction',action)

## create the act
man = xosc.Maneuver('cutInManeuver')
man.add_event(event)
mangr = xosc.ManeuverGroup('cutInMangroup')
mangr.add_actor('$owner')
mangr.add_maneuver(man)
starttrigger = xosc.ValueTrigger('cutinStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
act = xosc.Act('cutInct',starttrigger)
act.add_maneuver_group(mangr)

## create the story
storyparam = xosc.ParameterDeclarations()
storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
story = xosc.Story('cutInStory',storyparam)
story.add_act(act)

## create the storyboard
sb = xosc.StoryBoard(init)
sb.add_story(story)

## create the scenario
sce = xosc.Scenario('cutinScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

# Print the resulting xml
prettyprint(sce.get_element())

# write the OpenSCENARIO file as xosc using current script name
sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))

# uncomment the following lines to display the scenario using esmini
# from scenariogeneration import esmini
# esmini(sce,os.path.join('esmini'))
