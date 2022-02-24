import os
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import json
import math
import numpy as np

#-----------------------------------esmini data------------------------------
path_to_save_dir = 'plots/'
_file = 'esmini_real_plot_data.json'
with open(_file) as f:
    data = json.loads(f.read())

print("total number of scenarios: {}".format(len(data['data'])))

no = 0 
key = 'speed'
adversary_esmini3 = data['data'][no]['adversary_esmini']
adversary_real3 = data['data'][no]['adversary_real']
ego_esmini3 = data['data'][no]['ego_esmini']
ego_real3 = data['data'][no]['ego_real']

adversary_esmini_sec3 = data['data'][no]['adversary_esmini_sec']
adversary_real_sec3 = data['data'][no]['adversary_real_sec']
ego_esmini_sec3 = data['data'][no]['ego_esmini_sec']
ego_real_sec3 = data['data'][no]['ego_real_sec']

name = path_to_save_dir+"test"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+" (km/hr)")
plt.xlim([0, 11])
#plt.plot(adversary_real1['sec'], adversary_real1[key])
#plt.plot(adversary_esmini1['sec'], adversary_esmini1[key], '--')
#plt.plot(adversary_real2['sec'], adversary_real2[key])
#plt.plot(adversary_esmini2['sec'], adversary_esmini2[key], '--')
plt.plot(adversary_real_sec3['sec'], adversary_real_sec3[key])
plt.plot(adversary_esmini_sec3['sec'], adversary_esmini_sec3[key], '--')
plt.legend(['Real-world', 'OpenSCENARIO'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_ego"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+' (km/hr)')
#plt.ylim([0, 20])
plt.plot(ego_real_sec3['sec'], ego_real_sec3[key], label='real')
plt.plot(ego_esmini_sec3['sec'], ego_esmini_sec3[key], label='generated')
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


adversary_esmini_milli = data['data'][no]['adversary_esmini_milli']
adversary_real_milli = data['data'][no]['adversary_real_milli']

sec = []
speed = []
squared_error_s = []
squared_error_t = []
mse_sec = []
for index in range(len(adversary_esmini_sec3['speed'])):
    print("----------index: {}---------".format(index))
    sec.append(index*10)
    speed.append(adversary_esmini_sec3['speed'][index])
    if index < len(adversary_real_sec3['t']):
        '''
        x1 = adversary_real_sec3['t'][index]
        y1 = adversary_real_sec3['s'][index]
        x2 = adversary_esmini_sec3['t'][index]
        y2 = adversary_esmini_sec3['s'][index]
        error =  math.sqrt(np.square(x1-x2)+np.square(y1-y2))
        #y_pred = adversary_esmini_sec3['t'][index]
        #y_actual = adversary_real_sec3['t'][index]
        #squared_error = np.square(np.subtract(y_actual,y_pred))
        mse_data.append(error)
        '''
        s_actual = adversary_real_sec3['s'][index]
        s_pred = adversary_esmini_sec3['s'][index]
        t_actual = adversary_real_sec3['t'][index]
        t_pred = adversary_esmini_sec3['t'][index]
        squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
        squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
        mse_sec.append(index)
        print(np.square(np.subtract(s_actual,s_pred)))
        
rmse_s = math.sqrt(sum(squared_error_s)/len(squared_error_s))
rmse_t = math.sqrt(sum(squared_error_t)/len(squared_error_t))
print("rmse_s: {}, rmse_t:{}".format(rmse_s, rmse_t))

name = path_to_save_dir+"test_rmse_s"
plt.figure()
plt.xlabel("error")
plt.ylabel("second")
plt.plot(mse_sec, squared_error_s)
plt.legend(['rmse_s'])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_rmse_t"
plt.figure()
plt.xlabel("error")
plt.ylabel("second")
plt.plot(mse_sec, squared_error_t)
plt.legend(['rmse_t'])
plt.savefig(name)
plt.close()





sec_real = []
speed_real = []
for index in range(len(adversary_real_milli['speed'])):
    sec_real.append(index)
    speed_real.append(adversary_real_milli['speed'][index])

name = path_to_save_dir+"test_other_milli"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+' (km/hr)')
#plt.ylim([0, 20])
#plt.plot(adversary_esmini_milli['speed'], label='generated')
plt.plot(adversary_esmini_milli['sec'], adversary_esmini_milli['speed'], label='real')
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.legend(['Real-world', 'OpenSCENARIO'])
plt.savefig(name)

key = 's'
name = path_to_save_dir+"test_s_t"
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([0, -7])
plt.text(-5, adversary_real3[key][len(adversary_real3[key])-5], "rmse_s = {:.3f} m".format(rmse_s), fontsize = 10)
plt.text(-5, adversary_real3[key][len(adversary_real3[key])-15], "rmse_t = {:.3f} m".format(rmse_t), fontsize = 10)
#plt.ylim([100, 130])
#plt.plot(adversary_real1['sec'], adversary_real1[key])
#plt.plot(adversary_esmini1['sec'], adversary_esmini1[key], '--')
#plt.plot(adversary_real2['sec'], adversary_real2[key])
#plt.plot(adversary_esmini2['sec'], adversary_esmini2[key], '--')
plt.plot(adversary_real3['t'], adversary_real3[key])
plt.plot(adversary_esmini3['t'], adversary_esmini3[key], '--')
plt.legend(['Real-world', 'OpenSCENARIO'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()



'''
name = path_to_save_dir+"test_rss"
key = 'rss'
plt.figure()
plt.xlabel("second")
plt.ylabel(key)
plt.ylim([10, 40])
#plt.plot(esmini1['sec'], esmini1[key], '--')
#plt.plot(real1['sec'], real1[key])
#plt.plot(esmini2['sec'], esmini2[key], '--')
#plt.plot(real2['sec'], real2[key])
plt.plot(esmini3['sec'], esmini3[key], '--')
plt.plot(real3['sec'], real3[key])
plt.legend(['RSS computed in OpenSCENARIO', 'RSS computed in real data'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''



