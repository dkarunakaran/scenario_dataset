import os
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import json

'''
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
'''
#-----------------------------------esmini data------------------------------
path_to_save_dir = 'plots/'
_file = 'esmini_real_plot_data.json'
with open(_file) as f:
    data = json.loads(f.read())

print("total number of scenarios: {}".format(len(data['data'])))

no = 7
key = 'speed'
adversary_esmini1 = data['data'][no]['adversary_esmini'] 
adversary_real1 = data['data'][no]['adversary_real'] 
ego_esmini1 = data['data'][no]['ego_esmini']
ego_real1 = data['data'][no]['ego_real']
esmini1 = data['data'][no]['esmini'] 
real1 = data['data'][no]['real'] 

no = 3
key = 'speed'
adversary_esmini2 = data['data'][no]['adversary_esmini'] 
adversary_real2 = data['data'][no]['adversary_real'] 
ego_esmini2 = data['data'][no]['ego_esmini'] 
ego_real2 = data['data'][no]['ego_real'] 
esmini2 = data['data'][no]['esmini'] 
real2 = data['data'][no]['real'] 

no = 2
key = 'speed'
adversary_esmini3 = data['data'][no]['adversary_esmini']
adversary_real3 = data['data'][no]['adversary_real']
ego_esmini3 = data['data'][no]['ego_esmini']
ego_real3 = data['data'][no]['ego_real']
esmini3 = data['data'][no]['esmini'] 
real3 = data['data'][no]['real'] 


name = path_to_save_dir+"test"
plt.figure()
plt.xlabel("second")
plt.ylabel(key)
plt.ylim([2.5, 17.5])
plt.plot(adversary_esmini1['sec'], adversary_esmini1[key], '--')
plt.plot(adversary_real1['sec'], adversary_real1[key])
plt.plot(adversary_esmini2['sec'], adversary_esmini2[key], '--')
plt.plot(adversary_real2['sec'], adversary_real2[key])
plt.plot(adversary_esmini3['sec'], adversary_esmini3[key], '--')
plt.plot(adversary_real3['sec'], adversary_real3[key])
plt.legend(['OpenSCENARIO trajectory', 'Real-world trajectory'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_ego"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+' m/s')
plt.ylim([0, 20])
plt.plot(ego_esmini1['sec'], ego_esmini1[key], label='generated')
plt.plot(ego_real1['sec'], ego_real1[key], label='real')
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


name = path_to_save_dir+"test_rss"
key = 'rss'
plt.figure()
plt.xlabel("second")
plt.ylabel(key)
plt.ylim([10, 40])
plt.plot(esmini1['sec'], esmini1[key], '--')
plt.plot(real1['sec'], real1[key])
plt.plot(esmini2['sec'], esmini2[key], '--')
plt.plot(real2['sec'], real2[key])
plt.plot(esmini3['sec'], esmini3[key], '--')
plt.plot(real3['sec'], real3[key])
plt.legend(['RSS computed in OpenSCENARIO', 'RSS computed in real data'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


