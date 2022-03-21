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
_file = 'esmini_plot_data_complete.json'
with open(_file) as f:
    data = json.loads(f.read())

print("total number of scenarios: {}".format(len(data['data'])))


points = [0,1] 
no = points[0]


#---------s_t--------------

real_data = data['data'][no]
ego_real_sec = real_data['ego_real_sec']
ego_esmini_sec = real_data['ego_esmini_sec']
adversary_real = real_data['adversary_real']
adversary_real_sec = real_data['adversary_real_sec']
adversary_real_sec['cut_time'] = 4

s_ego_real_sec = []
for index in range(len(ego_real_sec['s'])):
    if index == 0:
        s_ego_real_sec.append(0)
        s_prev = 0
    else:
        s_diff = ego_real_sec['s'][index] - ego_real_sec['s'][index-1]
        s_current = s_prev+s_diff
        s_ego_real_sec.append(s_current)
        s_prev = s_current


t_ego_real_sec = []
for index in range(len(ego_real_sec['s'])):
    t_ego_real_sec.append(ego_real_sec['t'][index])

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


s_real_sec = []
for index in range(len(adversary_real_sec['s'])):
    if index == 0:
        s_real_sec.append(0)
        s_prev = 0
    else:
        s_diff = adversary_real_sec['s'][index] - adversary_real_sec['s'][index-1]
        s_current = s_prev+s_diff
        s_real_sec.append(s_current)
        s_prev = s_current

s_cut_real = []
t_cut_real = []
for index in range(len(s_real_sec)):
    if index < adversary_real_sec['cut_time']:
        continue
    s_cut_real.append(s_real_sec[index])
    t_cut_real.append(adversary_real_sec['t'][index])



s_ego_esmini_sec = []
for index in range(len(ego_esmini_sec['s'])):
    if index == 0:
        s_ego_esmini_sec.append(0)
        s_prev = 0
    else:
        s_diff = ego_esmini_sec['s'][index] - ego_esmini_sec['s'][index-1]
        s_current = s_prev+s_diff
        s_ego_esmini_sec.append(s_current)
        s_prev = s_current

t_ego_esmini_sec = []
for index in range(len(ego_esmini_sec['s'])):
    t_ego_esmini_sec.append(ego_esmini_sec['t'][index])


adversary_esmini_all = {}
for point in points: 
    print("Point: {}".format(point))
    adv_data = data['data'][point]
    ego_esmini_sec = adv_data['ego_esmini_sec']
    adversary_esmini = adv_data['adversary_esmini']
    adversary_esmini_sec = adv_data['adversary_esmini_sec']
    adversary_esmini_sec['cut_time'] = 5
    if point == 1:
        adversary_esmini_sec['cut_time'] = 4
    s_cut_diff = adversary_esmini_sec['adv_real_esmini_s_diff']
    print("s_cut_diff: {}".format(s_cut_diff))
    adversary_esmini_all[point] = None
    s_gen = []
    for index in range(len(adversary_esmini['s'])):
        if index == 0:
            s_gen.append(0)
            s_prev = 0
        else:
            s_diff = adversary_esmini['s'][index] - adversary_esmini['s'][index-1]
            s_current = s_prev+s_diff
            s_gen.append(s_current)
            s_prev = s_current
    
    s_pred_main = []
    t_pred_main = []
    for index in range(len(s_gen)):
        if (s_gen[index]+s_cut_diff) < 0:
            continue
        else:
            s_pred_main.append(s_gen[index]+s_cut_diff)
            t_pred_main.append(adversary_esmini['t'][index])

    s_esmini_sec = []
    for index in range(len(adversary_esmini_sec['s'])):
        if index == 0:
            s_esmini_sec.append(0+s_cut_diff)
            s_prev = 0
        else:
            s_diff = adversary_esmini_sec['s'][index] - adversary_esmini_sec['s'][index-1]
            s_current = s_prev+s_diff
            s_esmini_sec.append(s_current+s_cut_diff)
            s_prev = s_current

    

    s_cut_esmini = []
    t_cut_esmini = []
    for index in range(len(s_esmini_sec)):
        if index < adversary_esmini_sec['cut_time']:
            continue
        s_cut_esmini.append(s_esmini_sec[index])
        t_cut_esmini.append(adversary_esmini_sec['t'][index])
        

    sec = []
    speed = []
    squared_error_s = []
    squared_error_t = []
    squared_error_v = []
    mse_sec = []
    rel_long = []
    '''
    adversary_esmini_sec = adv_data['adversary_esmini_sec']
    for index in range(len(adversary_esmini_sec['s'])):
        print("----------index: {}---------".format(index))
        sec.append(index*10)
        speed.append(adversary_esmini_sec['s'][index])
        if index < len(adversary_real_sec['s']):
            s_actual = adversary_real_sec['s'][index]
            s_pred = adversary_esmini_sec['s'][index]
            rel_long.append(s_actual-s_pred)
            #rel_long.append(s_pred-s_actual)
            t_actual = adversary_real_sec['t'][index]
            t_pred = adversary_esmini_sec['t'][index]
            v_actual = adversary_real_sec['speed'][index] 
            v_pred = adversary_esmini_sec['speed'][index] 
            squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
            squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
            squared_error_v.append(np.square(np.subtract(v_actual,v_pred)))
            mse_sec.append(index)
            
    rmse_s = math.sqrt(sum(squared_error_s)/len(squared_error_s))
    rmse_t = math.sqrt(sum(squared_error_t)/len(squared_error_t))
    rmse_v = math.sqrt(sum(squared_error_v)/len(squared_error_v))
    print("rmse_v: {}, rmse_s: {}, rmse_t:{}".format(rmse_v, rmse_s, rmse_t))
    '''
    for index in range(len(s_cut_real)):
        s_actual = s_cut_real[index]
        s_pred = s_cut_esmini[index]
        t_actual = t_cut_real[index]
        t_pred = t_cut_esmini[index]
        rel_long.append(s_pred-s_actual)
        squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
        squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
        mse_sec.append(index)

    rmse_s = math.sqrt(sum(squared_error_s)/len(squared_error_s))
    rmse_t = math.sqrt(sum(squared_error_t)/len(squared_error_t))
    print("rmse_s: {}, rmse_t:{}".format(rmse_s, rmse_t))

    adv_data = {
        's_esmini_sec': s_esmini_sec,
        't_esmini_sec': adversary_esmini_sec['t'],
        's': s_pred_main,
        't': t_pred_main,
        'rmse_s': rmse_s,
        'rmse_t': rmse_t,
        'rel_long': rel_long,
        'mse_sec': mse_sec,
        'cut-time': adv_data['adversary_esmini_sec']['cut_time'],
        'rss': adv_data['adversary_esmini_sec']['rss'],
        'rel_long': adv_data['adversary_esmini_sec']['rel_long']
    }

    adversary_esmini_all[point] = adv_data

name = path_to_save_dir+"scenario_s_t_2_point"+str(no)
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([-1, -5])
font_size = 9
plt.text(-2.25, 15, "2 points: rmse_s = {:.2f} m and rmse_t = {:.2f} m".format(adversary_esmini_all[points[0]]['rmse_s'],adversary_esmini_all[points[0]]['rmse_t']), fontsize = font_size)
#plt.text(-2.25, 10, "4 points: rmse_s = {:.2f} m and rmse_t = {:.2f} m".format(adversary_esmini_all[points[1]]['rmse_s'],adversary_esmini_all[points[1]]['rmse_t']),fontsize = font_size)
plt.plot(adversary_real['t'], s_real, color='b')
plt.plot(adversary_esmini_all[points[0]]['t'], adversary_esmini_all[points[0]]['s'],'--', color='g')
#plt.plot(adversary_esmini_all[points[1]]['t'], adversary_esmini_all[points[1]]['s'],'--',color='r')
for index in range(len(s_real_sec)): 
    if index >= adversary_real_sec['cut_time']-1:
        plt.text(adversary_real_sec['t'][index],s_real_sec[index],str((index+2)-adversary_real_sec['cut_time']),fontsize=10,color='w')
        plt.plot(adversary_real_sec['t'][index]-0.05,s_real_sec[index]+1,marker='o',markersize=10,color='b')

for index in range(len(s_real_sec)): 
    if index >= adversary_esmini_all[points[0]]['cut-time']-1:
        plt.text(adversary_esmini_all[points[0]]['t_esmini_sec'][index],adversary_esmini_all[points[0]]['s_esmini_sec'][index],str((index+2)-adversary_esmini_all[points[0]]['cut-time']),fontsize=10,color='w')
        plt.plot(adversary_esmini_all[points[0]]['t_esmini_sec'][index]-0.05,adversary_esmini_all[points[0]]['s_esmini_sec'][index]+1,marker='o',markersize=10,color='g')
'''
for index in range(len(s_real_sec)): 
    if index >= adversary_esmini_all[points[1]]['cut-time']-1:
        plt.text(adversary_esmini_all[points[1]]['t_esmini_sec'][index],adversary_esmini_all[points[1]]['s_esmini_sec'][index],str((index+2)-adversary_esmini_all[points[1]]['cut-time']),fontsize=10,color='r')
 
plt.legend(['Real-world', '2 points', '4 points'])
'''
plt.legend(['Real-world', '2 points'])
plt.savefig(name)
plt.close()


name = path_to_save_dir+"scenario_s_t_4_point"+str(no)
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([-1, -5])
font_size = 9
plt.text(-2.25, 10, "4 points: rmse_s = {:.2f} m and rmse_t = {:.2f} m".format(adversary_esmini_all[points[1]]['rmse_s'],adversary_esmini_all[points[1]]['rmse_t']),fontsize = font_size)
plt.plot(adversary_real['t'], s_real, color='b')
plt.plot(adversary_esmini_all[points[1]]['t'], adversary_esmini_all[points[1]]['s'],'--',color='r')
for index in range(len(s_real_sec)): 
    if index >= adversary_real_sec['cut_time']-1:
        plt.text(adversary_real_sec['t'][index],s_real_sec[index],str((index+2)-adversary_real_sec['cut_time']),fontsize=10,color='w')
        plt.plot(adversary_real_sec['t'][index]-0.05,s_real_sec[index]+1,marker='o',markersize=10,color='b')

for index in range(len(s_real_sec)): 
    if index >= adversary_esmini_all[points[1]]['cut-time']-2:
        plt.text(adversary_esmini_all[points[0]]['t_esmini_sec'][index],adversary_esmini_all[points[0]]['s_esmini_sec'][index],str((index+2)-adversary_esmini_all[points[0]]['cut-time']),fontsize=10,color='w')
        plt.plot(adversary_esmini_all[points[0]]['t_esmini_sec'][index]-0.05,adversary_esmini_all[points[0]]['s_esmini_sec'][index]+1,marker='o',markersize=10,color='r')
 
plt.legend(['Real-world', '4 points'])
plt.savefig(name)
plt.close()


mse_sec_modified_0 = []
for index in range(len(adversary_esmini_all[points[0]]['mse_sec'])):
    if index > 5:
        continue
    mse_sec_modified_0.append(adversary_esmini_all[points[0]]['mse_sec'][index])


mse_sec_modified_1 = []
for index in range(len(adversary_esmini_all[points[1]]['mse_sec'])):
    if index > 5:
        continue
    mse_sec_modified_1.append(adversary_esmini_all[points[1]]['mse_sec'][index])

rel_long_modified_0 = []
for index in range(len(adversary_esmini_all[points[0]]['rel_long'])):
    if index > 5:
        continue
    rel_long_modified_0.append(adversary_esmini_all[points[0]]['rel_long'][index])

rel_long_modified_1 = []
for index in range(len(adversary_esmini_all[points[1]]['rel_long'])):
    if index > 5:
        continue

    rel_long_modified_1.append(adversary_esmini_all[points[1]]['rel_long'][index])



name = path_to_save_dir+"scenario_rel_long_"+str(no)
plt.figure()
plt.ylabel("relative longitudinal position")
plt.xlabel("second")
plt.xlim([0, 6])
plt.plot(mse_sec_modified_0,rel_long_modified_0,color='r')
plt.plot(mse_sec_modified_1,rel_long_modified_1,color='g')
plt.legend(['4 points', '2 points'])
plt.savefig(name)
plt.close()


name = path_to_save_dir+"scenario_rss_"+str(no)
key = 'rss'
plt.figure()
plt.xlabel("second")
plt.ylabel(key)
plt.plot(adversary_esmini_all[points[0]][key],color='c')
plt.plot(adversary_esmini_all[points[0]]['rel_long'],'--', color='c')
plt.plot(adversary_esmini_all[points[1]][key], color='r')
plt.plot(adversary_esmini_all[points[1]]['rel_long'],'-.', color='r')
plt.legend(['RSS - 4 points', 'Long dist-4 points', 'RSS - 4 points with noice', 'Long dist - 4 points with noice'])
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
