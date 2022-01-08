from scenariogeneration import xodr
from scenariogeneration import xosc, prettyprint
import os
import json
import sys

class Generate:
    def __init__(self):
        self.save_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_files/'
    
    def generate_openscenario(self, _file, filename):
        with open(_file) as f:
            data = json.loads(f.read())
        print("Processing the file: {}".format(_file))
        _type = data['type']
        if _type == 'cutin':
            self.cutinScenario(_file, filename, data)
        if _type == 'cutout':
            self.cutOutScenario(_file, filename, data)
    
    def cutOutScenario(self, _file, filename, data):
        # GLOBAL PARAMETERS
        param_adv_percentage_dist_to_cutout_dist = 100
        param_ego_percentage_dist_to_cutout_dist = 100
        _type = data['type']
        count = 1
        for param in data['parameter']:
            name = self.save_loc+filename+"_"+_type+"_"+str(count)+".xosc"
            print(param)

            ### create catalogs
            catalog = xosc.Catalog()
            catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

            road = xosc.RoadNetwork(roadfile=filename+'.xodr')

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
            adversary_cutout_speed = 40 # param 7 
            adversary_cutout_trigger_dist = param['param_cutout_start_dist'] # param 8 - distance in s from target to ego
            adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2

            adversary_cutout_time = param['param_cutout_time'] # Param 10 time taken to complete the lane change
            adversary_to_cutout_time = param['param_adv_to_cutout_time']
            adversary_to_cutout_dist = param['param_adv_to_cutout_dist']
            adversary_avg_cutout_speed = param['param_adv_avg_cutout_speed']
            ego_to_cutout_dist = param['param_ego_to_cutout_dist']
            ego_to_cutout_time = param['param_ego_to_cutout_time']
            ego_avg_cutout_speed = param['param_ego_avg_cutout_speed']
            trigger_cond = param['param_trigger_cond']
            adversary_lane_final = param['param_adv_lane_no_final'] # param 6 - lane number
           
            paramdec = xosc.ParameterDeclarations()
            paramdec.add_parameter(xosc.Parameter('$egoVehicle',xosc.ParameterType.string, ego_vehicle))
            paramdec.add_parameter(xosc.Parameter('$egoVelocityInit',xosc.ParameterType.double, ego_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$egoStartPos',xosc.ParameterType.double, ego_start_pos))
            paramdec.add_parameter(xosc.Parameter('$egoStartLane',xosc.ParameterType.double, ego_start_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryVehicle',xosc.ParameterType.string, adversary_vehicle))
            paramdec.add_parameter(xosc.Parameter('$adversaryVelocityInit',xosc.ParameterType.double, adversary_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$adversaryPos',xosc.ParameterType.double, adversary_pos))
            paramdec.add_parameter(xosc.Parameter('$adversaryLane',xosc.ParameterType.integer,adversary_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutSpeed',xosc.ParameterType.double,adversary_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$adversaryStartDiff',xosc.ParameterType.double,adversary_start_diff))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutTriggerDist',xosc.ParameterType.double,adversary_cutout_trigger_dist))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutTime',xosc.ParameterType.double,adversary_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutDist',xosc.ParameterType.double,adversary_to_cutout_dist))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutTime',xosc.ParameterType.double,adversary_to_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$adversaryAvgCutoutSpeed',xosc.ParameterType.double,adversary_avg_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutDist',xosc.ParameterType.double,ego_to_cutout_dist))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutTime',xosc.ParameterType.double, ego_to_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$egoAvgCutoutSpeed',xosc.ParameterType.double, ego_avg_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$adversaryLaneFinal',xosc.ParameterType.integer,adversary_lane_final))

            
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

            # create event and act1 - cutout
            trig_cond1 = None
            if trigger_cond == 0:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutoutTriggerDist',xosc.Rule.lessThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            else:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutoutTriggerDist',xosc.Rule.greaterThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            trigger = xosc.EntityTrigger('cutOutTrigger',0,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutOnevent',xosc.Priority.overwrite)
            event.add_trigger(trigger)

            sin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.sinusoidal,xosc.DynamicsDimension.time,'$adversaryCutoutTime')
            action = xosc.AbsoluteLaneChangeAction('$adversaryLaneFinal',sin_time)
            event.add_action('cutOutLaneAction',action)
            man = xosc.Maneuver('cutOutManeuver')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroup')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            act = xosc.Act('cutOutAct',starttrigger)
            act.add_maneuver_group(mangr)
           
            # create event and act 2 - adversary vehicle
            adv_start_to_cutout_dist = (adversary_to_cutout_dist*param_adv_percentage_dist_to_cutout_dist)/100
            trig_cond1 = xosc.TraveledDistanceCondition(adv_start_to_cutout_dist)
            trigger = xosc.EntityTrigger('cutOutTriggerAdv',0.2,xosc.ConditionEdge.rising,trig_cond1,'$adversaryVehicle')
            event = xosc.Event('cutOuteventAdv',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time,'$adversaryToCutoutTime')
            action = xosc.AbsoluteSpeedAction('$adversaryAvgCutoutSpeed',lin_time)
            event.add_action('cutOutSpeedActionAdv',action)
           
            man = xosc.Maneuver('cutOutManeuverAdv')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupAdv')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerAdv',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actAdv = xosc.Act('cutOutActAdv',starttrigger)
            actAdv.add_maneuver_group(mangr)

            # create event and act 3 - ego vehicle
            if ego_to_cutout_dist > 20:
                ego_start_to_cutout_dist = 20
            else:
                ego_start_to_cutout_dist = (ego_to_cutout_dist*param_ego_percentage_dist_to_cutout_dist)/100
            trig_cond1 = xosc.TraveledDistanceCondition(ego_start_to_cutout_dist)
            trigger = xosc.EntityTrigger('cutOutTriggerEgo',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutOuteventEgo',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time,'$egoToCutoutTime')
            action = xosc.AbsoluteSpeedAction('$egoAvgCutoutSpeed',lin_time)
            event.add_action('cutOutSpeedActionEgo',action)
            man = xosc.Maneuver('cutOutManeuverEgo')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupEho')
            mangr.add_actor('$egoVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerEgo',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actEgo = xosc.Act('cutOutActEgo',starttrigger)
            actEgo.add_maneuver_group(mangr)

            ## create the story
            #storyparam = xosc.ParameterDeclarations()
            #storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
            #story = xosc.Story('cutInStory',storyparam)
            story = xosc.Story('cutOutStory')
            story.add_act(act)
            story.add_act(actAdv)
            story.add_act(actEgo)

            ## create the storyboard
            sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(30,xosc.Rule.greaterThan),'stop'))
            sb.add_story(story)

            ## create the scenario
            sce = xosc.Scenario('cutOutScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

            # Print the resulting xml
            #prettyprint(sce.get_element())

            # write the OpenSCENARIO file as xosc using current script name
            #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
            sce.write_xml(name)
            count += 1

    
    def cutinScenario(self, _file, filename, data):

        # GLOBAL PARAMETERS
        param_adv_percentage_dist_to_cutin_dist = 100
        param_ego_percentage_dist_to_cutin_dist = 100
        _type = data['type']
        count = 1
        for param in data['parameter']:
            name = self.save_loc+filename+"_"+_type+"_"+str(count)+".xosc"
            print(param)

            ### create catalogs
            catalog = xosc.Catalog()
            catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

            road = xosc.RoadNetwork(roadfile=filename+'.xodr')

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

            # Create event and act 4
            trig_cond1 = xosc.TimeToCollisionCondition(1,xosc.Rule.lessThan,entity='$adversaryVehicle')
            trigger = xosc.EntityTrigger('TTCTriggerEgo',0,xosc.ConditionEdge.none,trig_cond1,'$egoVehicle')
            event = xosc.Event('TTCeventEgo',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time, 1)
            action = xosc.AbsoluteSpeedAction('$adversaryAvgCutinSpeed',lin_time)
            event.add_action('TTCSpeedActionEgo',action)
            man = xosc.Maneuver('TTCManeuverEgo')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('TTCMangroupEho')
            mangr.add_actor('$egoVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('TTCStartTriggerEgo',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actTTC = xosc.Act('TTCActEgo',starttrigger)
            actTTC.add_maneuver_group(mangr)


            ## create the story
            #storyparam = xosc.ParameterDeclarations()
            #storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
            #story = xosc.Story('cutInStory',storyparam)
            story = xosc.Story('cutInStory')
            story.add_act(act)
            story.add_act(actAdv)
            story.add_act(actEgo)
            story.add_act(actTTC)

            ## create the storyboard
            sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(30,xosc.Rule.greaterThan),'stop'))
            sb.add_story(story)

            ## create the scenario
            sce = xosc.Scenario('cutinScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

            # Print the resulting xml
            #prettyprint(sce.get_element())

            # write the OpenSCENARIO file as xosc using current script name
            #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
            sce.write_xml(name)
            count += 1


       
    def generate_opendrive(self, _file, filename):
        with open(_file) as f:
            data = json.loads(f.read())
        print("Processing the file: {}".format(_file))

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
        noOfLanesStart = data['noOfLanesStart']
        for param in data['data']:
            noOfLanes = noOfLanesStart #param['noLanes']

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
        odr.write_xml(self.save_loc+filename+".xodr")



if __name__ == '__main__':
    gen = Generate()
    loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/'
    _dir = os.listdir(loc)
    for _file in _dir:
        base = os.path.basename(_file)
        fileDetails = os.path.splitext(base)
        if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
            gen.generate_openscenario(loc+_file, fileDetails[0])
            gen.generate_opendrive(loc+fileDetails[0]+".laneletToOD", fileDetails[0])
