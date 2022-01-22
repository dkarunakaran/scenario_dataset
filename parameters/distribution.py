import os
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import json

loc = '/model/parameters/'
_dir = os.listdir(loc)
evaluation = {
    'cut-in': {
        'param_ego_speed_init': [],
        'param_adv_speed_init': [],
        'param_start_diff': [],
        'param_cutin_start_dist': [],
        'param_cutin_time': [],
        'param_ego_to_cutin_dist': [],
        'param_ego_to_cutin_time': [],
        'param_adv_to_cutin_time': [],
        'param_adv_to_cutin_dist': []
    },
    'cut-out':{
        'param_ego_speed_init': [],
        'param_adv_speed_init': [],
        'param_start_diff': [],
        'param_cutout_start_dist': [],
        'param_cutout_time': [],
        'param_ego_to_cutout_dist': [],
        'param_ego_to_cutout_time': [],
        'param_adv_to_cutout_time': [],
        'param_adv_to_cutout_dist': []

    }
}
for _file in _dir:
    base = os.path.basename(_file)
    fileDetails = os.path.splitext(base)
    if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
        print("Processing the file: {}".format(_file))
        with open(_file) as f:
            data = json.loads(f.read())
        _type = data['type']
        if _type == 'cutin':
            for param in data['parameter']:
                evaluation['cut-in']['param_ego_speed_init'].append(param['param_ego_speed_init'])
                evaluation['cut-in']['param_adv_speed_init'].append(param['param_adv_speed_init'])
                evaluation['cut-in']['param_start_diff'].append(param['param_start_diff'])
                evaluation['cut-in']['param_cutin_start_dist'].append(param['param_cutin_start_dist'])
                evaluation['cut-in']['param_cutin_time'].append(param['param_cutin_time'])
                evaluation['cut-in']['param_ego_to_cutin_dist'].append(param['param_ego_to_cutin_dist'])
                evaluation['cut-in']['param_adv_to_cutin_time'].append(param['param_adv_to_cutin_time'])
                evaluation['cut-in']['param_adv_to_cutin_dist'].append(param['param_adv_to_cutin_dist'])
         
        if _type == 'cutout':
            for param in data['parameter']:
                evaluation['cut-out']['param_ego_speed_init'].append(param['param_ego_speed_init'])
                evaluation['cut-out']['param_adv_speed_init'].append(param['param_adv_speed_init'])
                evaluation['cut-out']['param_start_diff'].append(param['param_start_diff'])
                evaluation['cut-out']['param_cutout_start_dist'].append(param['param_cutout_start_dist'])
                evaluation['cut-out']['param_cutout_time'].append(param['param_cutout_time'])
                evaluation['cut-out']['param_ego_to_cutout_dist'].append(param['param_ego_to_cutout_dist'])
                evaluation['cut-out']['param_adv_to_cutout_time'].append(param['param_adv_to_cutout_time'])
                evaluation['cut-out']['param_adv_to_cutout_dist'].append(param['param_adv_to_cutout_dist'])
        
        

print(evaluation)

for key in evaluation['cut-in'].keys():
    path_to_save_dir = '/model/plots/param_plots/'
    name = path_to_save_dir+key
    plt.figure()
    hist = plt.hist(evaluation['cut-in'][key], bins='auto', color='#B45743', rwidth=0.25, normed=False)
    plt.ylabel("Frequency")
    plt.xlabel(key)
    plt.savefig(name)
    plt.close()


