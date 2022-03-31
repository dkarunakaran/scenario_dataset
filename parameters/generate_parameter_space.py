from scenariogeneration import xosc, prettyprint
import os
import json
import sys

class GenerateParamterSpace:
    def __init__(self):
        self.save_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/parameter_space/'
        self.cut_in = {
            'param_ego_lane_offset_init': [],
            'param_ego_lane_no_init': [],
            'param_ego_speed_init': [],
            'param_adv_lane_offset_start': [],
            'param_adv_lane_offset_end': [],
            'param_relative_lane_init': [],
            'param_adv_speed_init': [],
            'param_start_diff': [],
            'param_cut_triggering_dist': [],
            'param_cut_time': [],
            'param_cut_distance': [],
            'param_total_duration': [],
            'param_adv_to_cut_start_dist': [],
            'param_adv_to_cut_start_time': [],
            'param_adv_to_cut_start_speed': [],
            'param_adv_to_cutend_dist': [],
            'param_adv_cut_start_to_end_time': [],
            'param_adv_cutend_speed': [],
            'param_adv_to_scenario_end_dist': [],
            'param_adv_cutend_to_scenario_end_time': [],
            'param_adv_speed_final': []
        }
        self.cut_out = {
            'param_ego_lane_offset_init': [],
            'param_ego_lane_no_init': [],
            'param_ego_speed_init': [],
            'param_adv_lane_offset_start': [],
            'param_adv_lane_offset_end': [],
            'param_relative_lane_init': [],
            'param_adv_speed_init': [],
            'param_start_diff': [],
            'param_cut_triggering_dist': [],
            'param_cut_time': [],
            'param_cut_distance': [],
            'param_total_duration': [],
            'param_adv_to_cut_start_dist': [],
            'param_adv_to_cut_start_time': [],
            'param_adv_to_cut_start_speed': [],
            'param_adv_to_cutend_dist': [],
            'param_adv_cut_start_to_end_time': [],
            'param_adv_cutend_speed': [],
            'param_adv_to_scenario_end_dist': [],
            'param_adv_cutend_to_scenario_end_time': [],
            'param_adv_speed_final': [],
            'param_relative_lane_final': []

        }

    def relative_lane(self, ego_lane, adv_lane, origin):
        #positive for left side and negative for right side
        lane = 0 
        if ego_lane < adv_lane:
            lane = -1
        elif ego_lane > adv_lane:
            lane = +1

        print("Origin: {} - Relative lane: {} and ego_lane: {}".format(origin, lane, ego_lane))
        
        return lane

    def process(self, _file, filename):
        with open(_file) as f:
            all_data = json.loads(f.read())
        _type = all_data['type']
        print("type: {}".format(_type))
        for data in all_data['parameter']:
            if _type == 'cutin':
                self.cut_in['param_ego_lane_no_init'].append(data['param_ego_lane_no_init'])
                self.cut_in['param_ego_speed_init'].append(data['param_ego_speed_init'])
                self.cut_in['param_adv_lane_offset_start'].append(data['param_lane_offset_start'])
                self.cut_in['param_adv_lane_offset_end'].append(data['param_lane_offset_end'])
                self.cut_in['param_adv_speed_init'].append(data['param_adv_speed_init'])
                self.cut_in['param_start_diff'].append(data['param_start_diff'])
                #self.cut_in['param_adv_lane_no_init'].append(data['param_adv_lane_no_init'])
                self.cut_in['param_cut_triggering_dist'].append(data['param_cut_triggering_dist'])
                self.cut_in['param_cut_time'].append(data['param_cut_time'])
                self.cut_in['param_cut_distance'].append(data['param_cut_distance'])
                self.cut_in['param_total_duration'].append(data['param_total_duration'])
                self.cut_in['param_adv_to_cut_start_dist'].append(data['param_adv_to_cut_start_dist'])
                self.cut_in['param_adv_to_cut_start_time'].append(data['param_adv_to_cut_start_time'])
                self.cut_in['param_adv_to_cut_start_speed'].append(data['param_adv_to_cut_start_speed'])
                self.cut_in['param_adv_to_cutend_dist'].append(data['param_adv_to_cutend_dist'])
                self.cut_in['param_adv_cut_start_to_end_time'].append(data['param_adv_cut_start_to_end_time'])
                self.cut_in['param_adv_cutend_speed'].append(data['param_adv_cutend_speed'])
                self.cut_in['param_adv_to_scenario_end_dist'].append(data['param_adv_to_scenario_end_dist'])
                self.cut_in['param_adv_cutend_to_scenario_end_time'].append(data['param_adv_cutend_to_scenario_end_time'])
                self.cut_in['param_adv_speed_final'].append(data['param_adv_speed_final'])
                
                self.cut_in['param_relative_lane_init'] = self.relative_lane(data['param_ego_lane_no_init'], data['param_adv_lane_no_init'], 'cut_in init')
            elif _type == 'cutout':
                # cut_out 
                self.cut_out['param_ego_lane_no_init'].append(data['param_ego_lane_no_init'])
                self.cut_out['param_ego_speed_init'].append(data['param_ego_speed_init'])
                self.cut_out['param_adv_lane_offset_start'].append(data['param_lane_offset_start'])
                self.cut_out['param_adv_lane_offset_end'].append(data['param_lane_offset_end'])
                self.cut_out['param_adv_speed_init'].append(data['param_adv_speed_init'])
                self.cut_out['param_start_diff'].append(data['param_start_diff'])
                self.cut_out['param_cut_triggering_dist'].append(data['param_cut_triggering_dist'])
                self.cut_out['param_cut_time'].append(data['param_cut_time'])
                self.cut_out['param_cut_distance'].append(data['param_cut_distance'])
                self.cut_out['param_total_duration'].append(data['param_total_duration'])
                self.cut_out['param_adv_to_cut_start_dist'].append(data['param_adv_to_cut_start_dist'])
                self.cut_out['param_adv_to_cut_start_time'].append(data['param_adv_to_cut_start_time'])
                self.cut_out['param_adv_to_cut_start_speed'].append(data['param_adv_to_cut_start_speed'])
                self.cut_out['param_adv_to_cutend_dist'].append(data['param_adv_to_cutend_dist'])
                self.cut_out['param_adv_cut_start_to_end_time'].append(data['param_adv_cut_start_to_end_time'])
                self.cut_out['param_adv_cutend_speed'].append(data['param_adv_cutend_speed'])
                self.cut_out['param_adv_to_scenario_end_dist'].append(data['param_adv_to_scenario_end_dist'])
                self.cut_out['param_adv_cutend_to_scenario_end_time'].append(data['param_adv_cutend_to_scenario_end_time'])
                self.cut_out['param_adv_speed_final'].append(data['param_adv_speed_final'])
                self.cut_in['param_relative_lane_init'] = 0
                self.cut_in['param_relative_lane_final'] = self.relative_lane(data['param_ego_lane_no_init'], data['param_adv_lane_no_final'], 'cut_out init')

    def save(self):
        print("Saving the data")
        data = {
            'type': 'cutin',
            'data': self.cut_in
        }
        with open(self.save_loc+'cutin_space.json', 'w') as outfile:
            json.dump(data, outfile)
        
        data = {
            'type': 'cutout',
            'data': self.cut_out
        }
        with open(self.save_loc+'cutout_space.json', 'w') as outfile:
            json.dump(data, outfile)


if __name__ == '__main__':
    gen = GenerateParamterSpace()
    loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_data/'
    _dir = os.listdir(loc)
    for _file in _dir:
        base = os.path.basename(_file)
        fileDetails = os.path.splitext(base)
        if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
            print("file: {}".format(fileDetails[0]))
            gen.process(loc+_file, fileDetails[0])
            gen.save()    

