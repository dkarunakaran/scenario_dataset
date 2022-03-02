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


points = ['2','4'] 
data_no = 1
scenario = 1

real_data = data['data'][data_no]['2'][scenario]
adversary_real = real_data['adversary_real']
s_real = []
for index in range(len(adversary_real['s'])):
    if index == 0:
        s_real.append(0)
        s_prev = 0
    else:
        s_diff = adversary_real['s'][index] - adversary_real['s'][index-1]
        s_current = s_prev+s_diff
        s_real.append(s_current)
        s_prev = s_current

adversary_esmini_all = {}
for point in points: 
    print("Point: {}".format(point))
    adv_data = data['data'][data_no][point][scenario]
    adversary_esmini = adv_data['adversary_esmini']
    adversary_esmini_all[point] = None
    s_pred = []
    for index in range(len(adversary_esmini['s'])):
        if index == 0:
            s_pred.append(0)
            s_prev = 0
        else:
            s_diff = adversary_esmini['s'][index] - adversary_esmini['s'][index-1]
            s_current = s_prev+s_diff
            s_pred.append(s_current)
            s_prev = s_current
        
    adv_data = {
        's': s_pred,
        't': adversary_esmini['t']
    }

    adversary_esmini_all[point] = adv_data

name = path_to_save_dir+"test_s_t"
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([0, -7])
plt.plot(adversary_real['t'], s_real)
plt.plot(adversary_esmini_all['2']['t'], adversary_esmini_all['2']['s'],'--')
plt.plot(adversary_esmini_all['4']['t'], adversary_esmini_all['4']['s'],'--')
plt.legend(['Real-world', '2 points', '4 points'])
plt.savefig(name)
plt.close()



'''
key = 'speed'
data = data['data'][data_no][points][scenario]
adversary_esmini3 = data['adversary_esmini']
adversary_real3 = data['adversary_real']
ego_esmini3 = data['ego_esmini']
ego_real3 = data['ego_real']

adversary_esmini_sec3 = data['adversary_esmini_sec']
adversary_real_sec3 = data['adversary_real_sec']
ego_esmini_sec3 = data['ego_esmini_sec']
ego_real_sec3 = data['ego_real_sec']

#print(adversary_real_sec3)

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


adversary_esmini_milli = data['adversary_esmini_milli']
adversary_real_milli = data['adversary_real_milli']

sec = []
speed = []
squared_error_s = []
squared_error_t = []
squared_error_v = []
mse_sec = []
rel_long = []
for index in range(len(adversary_esmini_sec3['speed'])):
    print("----------index: {}---------".format(index))
    sec.append(index*10)
    speed.append(adversary_esmini_sec3['speed'][index])
    if index < len(adversary_real_sec3['t']):
        s_actual = adversary_real_sec3['s'][index]
        s_pred = adversary_esmini_sec3['s'][index]
        rel_long.append(s_actual-s_pred)
        t_actual = adversary_real_sec3['t'][index]
        t_pred = adversary_esmini_sec3['t'][index]
        v_actual = adversary_real_sec3['speed'][index] 
        v_pred = adversary_esmini_sec3['speed'][index] 
        squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
        squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
        squared_error_v.append(np.square(np.subtract(v_actual,v_pred)))
        mse_sec.append(index)
        
rmse_s = math.sqrt(sum(squared_error_s)/len(squared_error_s))
rmse_t = math.sqrt(sum(squared_error_t)/len(squared_error_t))
rmse_v = math.sqrt(sum(squared_error_v)/len(squared_error_v))
print("rmse_v: {}, rmse_s: {}, rmse_t:{}".format(rmse_v, rmse_s, rmse_t))

name = path_to_save_dir+"rel_long_s"
plt.figure()
plt.ylabel("relative longitudinal position")
plt.xlabel("second")
plt.plot(mse_sec, rel_long)
plt.legend([''])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_rmse_s"
plt.figure()
plt.ylabel("error")
plt.xlabel("second")
plt.plot(mse_sec, squared_error_s)
plt.legend(['rmse_s'])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_rmse_t"
plt.figure()
plt.ylabel("error")
plt.xlabel("second")
plt.plot(mse_sec, squared_error_t)
plt.legend(['rmse_t'])
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+" (km/hr)")
#plt.text(10,26, "rmse_v = {:.3f} km/hr".format(rmse_v), fontsize = 10)
#plt.xlim([0, 11])
plt.plot(adversary_real_sec3['sec'], adversary_real_sec3[key])
plt.plot(adversary_esmini_sec3['sec'], adversary_esmini_sec3[key], '--')
plt.legend(['Real-world', 'OpenSCENARIO'])
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
#plt.text(110,26, "rmse_v = {:.3f} km/hr".format(rmse_v), fontsize = 10)
#plt.ylim([0, 20])
plt.plot(sec_real, speed_real)
plt.plot(adversary_esmini_milli['sec'], adversary_esmini_milli['speed'],'--')
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.legend(['Real-world', 'OpenSCENARIO'])
plt.savefig(name)


s_real = []
for index in range(len(adversary_real3['s'])):
    if index == 0:
        s_real.append(0)
        s_prev = 0
    else:
        s_diff = adversary_real3['s'][index] - adversary_real3['s'][index-1]
        s_current = s_prev+s_diff
        s_real.append(s_current)
        s_prev = s_current

s_pred = []
for index in range(len(adversary_esmini3['s'])):
    if index == 0:
        s_pred.append(0)
        s_prev = 0
    else:
        s_diff = adversary_esmini3['s'][index] - adversary_esmini3['s'][index-1]
        s_current = s_prev+s_diff
        s_pred.append(s_current)
        s_prev = s_current




key = 's'
name = path_to_save_dir+"test_s_t"
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([0, -7])
#plt.text(-5, adversary_real3[key][len(adversary_real3[key])-30], "rmse_s = {:.3f} m".format(rmse_s), fontsize = 10)
#plt.text(-5, adversary_real3[key][len(adversary_real3[key])-40], "rmse_t = {:.3f} m".format(rmse_t), fontsize = 10)
#plt.plot(adversary_real3['t'], adversary_real3[key])
#plt.plot(adversary_esmini3['t'], adversary_esmini3[key], '--')
plt.plot(adversary_real3['t'], s_real)
#plt.plot(adversary_esmini3['t'], s_pred, '--')
#plt.legend(['Real-world', 'OpenSCENARIO'])
plt.savefig(name)
plt.close()

key = 's'
name = path_to_save_dir+"test_s_t_sec"
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([0, -7])
plt.plot(adversary_real_sec3['t'],adversary_real_sec3[key])
plt.plot(adversary_esmini_sec3['t'], adversary_esmini_sec3[key],'--')
plt.legend(['Real-world', 'OpenSCENARIO'])
plt.savefig(name)
plt.close()
'''
