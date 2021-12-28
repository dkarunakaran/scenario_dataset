import os
from scenariogeneration import xosc, prettyprint
import json

# GLOBAL PARAMETERS
param_adv_percentage_dist_to_cutin_dist = 100
param_ego_percentage_dist_to_cutin_dist = 100


file_loc='/home/beastan/Documents/phd/scenario_extraction/parameters/gate_south-end_north-end_20210608_054523.json'

save_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_files/'

with open(file_loc) as f:
    data = json.loads(f.read())
print("Processing the file: {}".format(file_loc))
count = 1
filename = os.path.basename(file_loc).split('.', 1)[0]
for param in data['parameter']:
    name = save_loc+filename+"_"+str(count)+".xosc"
    print(param)

    ### create catalogs
    catalog = xosc.Catalog()
    catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

    road = xosc.RoadNetwork(roadfile='road1.xodr')

    ### create parameters
    ego_vehicle = 'hero'
    ego_velocity_init = param['param_ego_speed_init'] # param 0
    ego_start_pos = param['param_ego_start_pos'] # param 1 - s coordinate of road frame
    ego_start_lane = param['param_ego_lane_no_init'] # param 2 - lane number
    adversary_vehicle = 'adversary'
    adversary_velocity_init = param['param_adv_speed_init'] # param 3
    # Difference between ego vehicle and adversary, positive value indicates the
    # Target location is ahead of ego vehicle
    adversary_pos = param['param_adv_start_pos'] # param 4 - front=0 or back=1
    adversary_start_diff = param['param_start_diff'] # param 5  - difference in s coordinate of road frame
    adversary_lane = param['param_adv_lane_no_init'] # param 6 - lane number
    adversary_cutin_speed = 40 # param 7 
    adversary_cutin_trigger_dist = param['param_cutin_start_dist'] # param 8 - distance in s from target to ego
    adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2

    adversary_cutin_time = param['param_cutin_time'] # Param 10 time taken to complete the lane change
    adversary_to_cutin_time = param['param_adv_to_cutin_time']
    adversary_to_cutin_dist = param['param_adv_to_cutin_dist']
    adversary_avg_cutin_speed = param['param_adv_avg_cutin_speed']
    ego_to_cutin_dist = param['param_ego_to_cutin_dist']
    ego_to_cutin_time = param['param_ego_to_cutin_time']
    ego_avg_cutin_speed = param['param_ego_avg_cutin_speed']
    trigger_cond = param['param_trigger_cond']
   
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
    paramdec.add_parameter(xosc.Parameter('$adversaryStartDiff',xosc.ParameterType.double,adversary_start_diff))
    paramdec.add_parameter(xosc.Parameter('$adversaryCutinTriggerDist',xosc.ParameterType.double,adversary_cutin_trigger_dist))
    paramdec.add_parameter(xosc.Parameter('$adversaryCutinTime',xosc.ParameterType.double,adversary_cutin_time))
    paramdec.add_parameter(xosc.Parameter('$adversaryToCutinDist',xosc.ParameterType.double,adversary_to_cutin_dist))
    paramdec.add_parameter(xosc.Parameter('$adversaryToCutinTime',xosc.ParameterType.double,adversary_to_cutin_time))
    paramdec.add_parameter(xosc.Parameter('$adversaryAvgCutinSpeed',xosc.ParameterType.double,adversary_avg_cutin_speed))
    paramdec.add_parameter(xosc.Parameter('$egoToCutinDist',xosc.ParameterType.double,ego_to_cutin_dist))
    paramdec.add_parameter(xosc.Parameter('$egoToCutinTime',xosc.ParameterType.double, ego_to_cutin_time))
    paramdec.add_parameter(xosc.Parameter('$egoAvgCutinSpeed',xosc.ParameterType.double, ego_avg_cutin_speed))

    
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
    
    # Adding carla controller
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

    # create event and act1 - cutin
    trig_cond1 = None
    if trigger_cond == 0:
        trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutinTriggerDist',xosc.Rule.lessThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
    else:
        trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutinTriggerDist',xosc.Rule.greaterThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
    trigger = xosc.EntityTrigger('cutinTrigger',0,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
    event = xosc.Event('cutInevent',xosc.Priority.overwrite)
    event.add_trigger(trigger)

    sin_time= xosc.TransitionDynamics(xosc.DynamicsShapes.sinusoidal,xosc.DynamicsDimension.time,'$adversaryCutinTime')
    action = xosc.AbsoluteLaneChangeAction('$egoStartLane',sin_time)
    event.add_action('cutInLaneAction',action)
    man = xosc.Maneuver('cutInManeuver')
    man.add_event(event)
    mangr = xosc.ManeuverGroup('cutInMangroup')
    mangr.add_actor('$adversaryVehicle')
    mangr.add_maneuver(man)
    starttrigger = xosc.ValueTrigger('cutinStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
    act = xosc.Act('cutInAct',starttrigger)
    act.add_maneuver_group(mangr)
   
    # create event and act 2 - adversary vehicle
    adv_start_to_cutin_dist = (adversary_to_cutin_dist*param_adv_percentage_dist_to_cutin_dist)/100
    trig_cond1 = xosc.TraveledDistanceCondition(adv_start_to_cutin_dist)
    trigger = xosc.EntityTrigger('cutinTriggerAdv',0.2,xosc.ConditionEdge.rising,trig_cond1,'$adversaryVehicle')
    event = xosc.Event('cutIneventAdv',xosc.Priority.parallel)
    event.add_trigger(trigger)
    lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time, '$adversaryToCutinTime')
    action = xosc.AbsoluteSpeedAction('$adversaryAvgCutinSpeed',lin_time)
    event.add_action('cutInSpeedActionAdv',action)
   
    man = xosc.Maneuver('cutInManeuverAdv')
    man.add_event(event)
    mangr = xosc.ManeuverGroup('cutInMangroupAdv')
    mangr.add_actor('$adversaryVehicle')
    mangr.add_maneuver(man)
    starttrigger = xosc.ValueTrigger('cutinStartTriggerAdv',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
    actAdv = xosc.Act('cutInActAdv',starttrigger)
    actAdv.add_maneuver_group(mangr)

    # create event and act 3 - ego vehicle
    if ego_to_cutin_dist > 20:
        ego_start_to_cutin_dist = 20
    else:
        ego_start_to_cutin_dist = (ego_to_cutin_dist*param_ego_percentage_dist_to_cutin_dist)/100
    trig_cond1 = xosc.TraveledDistanceCondition(ego_start_to_cutin_dist)
    trigger = xosc.EntityTrigger('cutinTriggerEgo',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
    event = xosc.Event('cutIneventEgo',xosc.Priority.parallel)
    event.add_trigger(trigger)
    lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time,'$egoToCutinTime')
    action = xosc.AbsoluteSpeedAction('$egoAvgCutinSpeed',lin_time)
    event.add_action('cutInSpeedActionEgo',action)
    man = xosc.Maneuver('cutInManeuverEgo')
    man.add_event(event)
    mangr = xosc.ManeuverGroup('cutInMangroupEho')
    mangr.add_actor('$egoVehicle')
    mangr.add_maneuver(man)
    starttrigger = xosc.ValueTrigger('cutinStartTriggerEgo',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
    actEgo = xosc.Act('cutInActEgo',starttrigger)
    actEgo.add_maneuver_group(mangr)

    ## create the story
    #storyparam = xosc.ParameterDeclarations()
    #storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
    #story = xosc.Story('cutInStory',storyparam)
    story = xosc.Story('cutInStory')
    story.add_act(act)
    story.add_act(actAdv)
    story.add_act(actEgo)

    ## create the storyboard
    sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(30,xosc.Rule.greaterThan),'stop'))
    sb.add_story(story)

    ## create the scenario
    sce = xosc.Scenario('cutinScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

    # Print the resulting xml
    prettyprint(sce.get_element())

    # write the OpenSCENARIO file as xosc using current script name
    #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
    sce.write_xml(name)
    count += 1

# uncomment the following lines to display the scenario using esmini
# from scenariogeneration import esmini
# esmini(sce,os.path.join('esmini'))
