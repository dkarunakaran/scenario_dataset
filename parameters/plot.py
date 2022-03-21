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
s_cut_diff = adversary_esmini_sec3['adv_real_esmini_s_diff']
print("s_cut_diff: {}".format(s_cut_diff))
print("cut-time real: {}, cut-time esmini: {}".format(adversary_real_sec3['cut_time'], adversary_esmini_sec3['cut_time']))

# No noise
#adversary_real_sec3['cut_time'] = 5 #5
#adversary_esmini_sec3['cut_time'] = 5 #5

# reducing 2 m/s
#adversary_real_sec3['cut_time'] = 5
#adversary_esmini_sec3['cut_time'] = 6

# adding 2 m/s
#adversary_real_sec3['cut_time'] = 5
#adversary_esmini_sec3['cut_time'] = 4 #5

# adding 4 m/s
adversary_real_sec3['cut_time'] = 5
adversary_esmini_sec3['cut_time'] = 4


#s_cut_diff = 0

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


s_real_sec = []
for index in range(len(adversary_real_sec3['s'])):
    if index == 0:
        s_real_sec.append(0)
        s_prev = 0
    else:
        s_diff = adversary_real_sec3['s'][index] - adversary_real_sec3['s'][index-1]
        s_current = s_prev+s_diff
        s_real_sec.append(s_current)
        s_prev = s_current

s_esmini_sec = []
for index in range(len(adversary_esmini_sec3['s'])):
    if index == 0:
        s_esmini_sec.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec3['s'][index] - adversary_esmini_sec3['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec.append(s_current+s_cut_diff)
        s_prev = s_current

s_cut_esmini = []
t_cut_esmini = []
s_cut_real = []
t_cut_real = []
for index in range(len(s_esmini_sec)):
    if index < adversary_esmini_sec3['cut_time']:
        continue
    s_cut_esmini.append(s_esmini_sec[index])
    t_cut_esmini.append(adversary_esmini_sec3['t'][index])

for index in range(len(s_real_sec)):
    if index < adversary_real_sec3['cut_time']:
        continue
    s_cut_real.append(s_real_sec[index])
    t_cut_real.append(adversary_real_sec3['t'][index])




sec = []
speed = []
squared_error_s = []
squared_error_t = []
squared_error_v = []
mse_sec = []
rel_long = []


'''
for index in range(len(adversary_esmini_sec3['speed'])):
    print("----------index: {}---------".format(index))
    sec.append(index*10)
    speed.append(adversary_esmini_sec3['speed'][index])
    
    if index < len(adversary_real_sec3['t']):
        s_actual = adversary_real_sec3['s'][index]
        s_pred = adversary_esmini_sec3['s'][index]
        rel_long.append(s_actual-s_pred)
        t_actual = adversary_real_sec3['t'][index]
      rt_size  t_pred = adversary_esmini_sec3['t'][index]
        v_actual = adversary_real_sec3['speed'][index] 
        v_pred = adversary_esmini_sec3['speed'][index] 
        squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
        squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
        squared_error_v.append(np.square(np.subtract(v_actual,v_pred)))
        mse_sec.append(index)
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
#rmse_v = math.sqrt(sum(squared_error_v)/len(squared_error_v))
print("rmse_s: {}, rmse_t:{}".format(rmse_s, rmse_t))

name = path_to_save_dir+"rel_long_s"
plt.figure()
plt.ylabel("relative longitudinal position")
plt.xlabel("second")
plt.plot(mse_sec, rel_long, color='g')
plt.legend(['2 points'])
plt.savefig(name)
plt.close()

rqs = []
for index in range(len(squared_error_s)):
    rqs.append(math.sqrt(squared_error_s[index]))

name = path_to_save_dir+"test_rmse_s"
plt.figure()
plt.ylabel("error")
plt.xlabel("second")
plt.plot(mse_sec, rqs)
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

temp_s_pred = s_pred
s_pred = []
t_pred = []
for index in range(len(temp_s_pred)):
    if (temp_s_pred[index]+s_cut_diff) < 0:
        continue
    else:
        s_pred.append(temp_s_pred[index]+s_cut_diff)
        t_pred.append(adversary_esmini3['t'][index]) 


s_real_sec = []
for index in range(len(adversary_real_sec3['s'])):
    if index == 0:
        s_real_sec.append(0)
        s_prev = 0
    else:
        s_diff = adversary_real_sec3['s'][index] - adversary_real_sec3['s'][index-1]
        s_current = s_prev+s_diff
        s_real_sec.append(s_current)
        s_prev = s_current

s_esmini_sec = []
for index in range(len(adversary_esmini_sec3['s'])):
    if index == 0:
        s_esmini_sec.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec3['s'][index] - adversary_esmini_sec3['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec.append(s_current+s_cut_diff)
        s_prev = s_current

key = 's'
name = path_to_save_dir+"test_s_t"
plt.figure()
plt.figure(figsize=(35,50)) #15 25 #35 60
plt.xlabel("t, lateral displacement", size=100)
plt.ylabel('s, longitudinal displacement',size=100)
plt.xlim([-1.0, -5.0])
plt.ylim([0, 140])
plt.text(-3, 21, "rmse_s = {:.3f} m".format(rmse_s), fontsize = 100)
plt.text(-3, 16, "rmse_t = {:.3f} m".format(rmse_t), fontsize = 100)
#plt.plot(adversary_real3['t'], adversary_real3[key])
#plt.plot(adversary_esmini3['t'], adversary_esmini3[key], '--')
plt.plot(adversary_real3['t'], s_real, color='b',  linewidth=20)
color = 'r'
plt.plot(t_pred, s_pred, '--', color=color, linewidth=20)
for index in range(len(s_real_sec)): 
    if index >= adversary_esmini_sec3['cut_time']-1:
        plt.text(adversary_esmini_sec3['t'][index],s_esmini_sec[index]-1.25,str((index+2)-adversary_esmini_sec3['cut_time']),fontsize=110, color='w')
        plt.plot(adversary_esmini_sec3['t'][index]-0.05,s_esmini_sec[index],marker='o',markersize=115,color=color)

for index in range(len(s_real_sec)): 
    if index >= adversary_real_sec3['cut_time']-1:
        plt.text(adversary_real_sec3['t'][index]+0.1,s_real_sec[index]-1.25,str((index+2)-adversary_real_sec3['cut_time']),fontsize=110, color='w')
        plt.plot(adversary_real_sec3['t'][index]+0.05,s_real_sec[index],marker='o',markersize=115, color='b')

plt.tick_params(axis='x', labelsize=100)
plt.tick_params(axis='y', labelsize=100)
plt.legend(['Real-world', 'OpenSCENARIO'], prop={'size':100})
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





rss_real = []
rss_esmini = []
rel_long = []
for index in range(len(adversary_real_sec3['rss'])):
    if index >= adversary_real_sec3['cut_time']-1:
        rss_real.append(adversary_real_sec3['rss'][index])

'''
s_cut_diff = 2
for index in range(len(adversary_esmini_sec3['rss'])):
    if index > 11:
        break

    if index >= 3: 
        rss_esmini.append(adversary_esmini_sec3['rss'][index]+s_cut_diff)
        rel_long.append(adversary_esmini_sec3['rel_long'][index]+s_cut_diff)
'''
sec = 1
secs = []

rss_diff = adversary_esmini_sec3['rss'][adversary_esmini_sec3['cut_time']-1] - adversary_real_sec3['rss'][adversary_real_sec3['cut_time']-1] 
#rss_diff_real_esmini = adversary_real_sec3['rss_real_ego_esmini_challenging'][adversary_real_sec3['cut_time']-1] - adversary_real_sec3['rss'][adversary_real_sec3['cut_time']-1] 
rel_long_diff = adversary_esmini_sec3['rel_long'][adversary_esmini_sec3['cut_time']-1]- adversary_real_sec3['rel_long'][adversary_real_sec3['cut_time']-1]

print(rss_diff)
print(rel_long_diff)
#print(rss_diff_real_esmini)
if rss_diff < 0:
    rss_diff *= -1

#if rss_diff_real_esmini < 0:
#    rss_diff_real_esmini *= -1

if rel_long_diff < 0:
    rel_long_diff *= -1

rss_real_ego_esmini_challenging = []

for index in range(len(adversary_esmini_sec3['rss'])):
    if index > 10:
        break
    if index >= adversary_esmini_sec3['cut_time']-1:
        rss = adversary_esmini_sec3['rss'][index]+rss_diff
        if rss < 4.0:
            rss = 4.0
        rss_esmini.append(rss)
        rel_long.append(adversary_esmini_sec3['rel_long'][index]+rel_long_diff)
        secs.append(sec)
        sec += 1
        #rss_real_ego_esmini_challenging.append(adversary_real_sec3['rss_real_ego_esmini_challenging'][index])


name = path_to_save_dir+"test_rss"
key = 'rss'
plt.figure()
plt.figure(figsize=(10,10))
plt.xlabel("second",size=25)
plt.ylabel("distance (m)",size=25)
plt.plot(secs, rss_real, color='b', linewidth=5)
plt.plot(secs, rss_esmini,'--',color='r', linewidth=5)
plt.tick_params(axis='x', labelsize=25)
plt.tick_params(axis='y', labelsize=25)
plt.legend(['RSS - real-world', 'RSS - generated scenario'], prop={'size':25})
plt.savefig(name)
plt.close()



name = path_to_save_dir+"rss_noise3"
key = 'rss'
plt.figure()
plt.figure(figsize=(9,9))
plt.ylim([14, 50])
plt.xlabel("second",size=30)
plt.ylabel("distance (m)",size=30)
plt.plot(secs, rss_esmini,color='y',linewidth=5)
plt.plot(secs, rel_long,color='k',linewidth=5)
plt.tick_params(axis='x', labelsize=30)
plt.tick_params(axis='y', labelsize=30)
plt.legend(['RSS distance', 'Relative distance'], prop={'size':25})
plt.savefig(name)
plt.close()


exit()

# Generated and generated with diturbance comparison
adversary_esmini_sec_without_dist = data['data'][0]['adversary_esmini_sec']
adversary_esmini_without_dist  = data['data'][0]['adversary_esmini']
cut_time_esmini_without_dist = 5

# First noise scneario
adversary_esmini_with_dist = data['data'][2]['adversary_esmini']
adversary_esmini_sec_with_dist = data['data'][2]['adversary_esmini_sec']
cut_time_esmini_with_dist = 6

# Second noise scenario
adversary_esmini_sec_with_dist2 = data['data'][4]['adversary_esmini_sec']
adversary_esmini_with_dist2 = data['data'][4]['adversary_esmini']
cut_time_esmini_with_dist2 = 4

# third noise scenario
adversary_esmini_sec_with_dist3 = data['data'][8]['adversary_esmini_sec']
adversary_esmini_with_dist3 = data['data'][8]['adversary_esmini']
cut_time_esmini_with_dist3 = 4



s_esmini_without_d = []
for index in range(len(adversary_esmini_without_dist['s'])):
    if index == 0:
        s_esmini_without_d.append(0)
        s_prev = 0
    else:
        s_diff = adversary_esmini_without_dist['s'][index] - adversary_esmini_without_dist['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_without_d.append(s_current)
        s_prev = s_current

s_cut_diff = adversary_esmini_sec_with_dist['adv_real_esmini_s_diff']
temp_s_pred = s_esmini_without_d
s_esmini_without_d = []
t_esmini_without_d = []
for index in range(len(temp_s_pred)):
    if (temp_s_pred[index]+s_cut_diff) < 0:
        continue
    else:
        s_esmini_without_d.append(temp_s_pred[index]+s_cut_diff)
        t_esmini_without_d.append(adversary_esmini_without_dist['t'][index]) 

s_esmini_sec_without_d = []
for index in range(len(adversary_esmini_sec_without_dist['s'])):
    if index == 0:
        s_esmini_sec_without_d.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec_without_dist['s'][index] - adversary_esmini_sec_without_dist['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec_without_d.append(s_current+s_cut_diff)
        s_prev = s_current


#first noise scenario
s_cut_diff = adversary_esmini_sec_with_dist['adv_real_esmini_s_diff']
print("s_cut_diff first noise scenario: {}".format(s_cut_diff))

s_esmini_with_d = []
for index in range(len(adversary_esmini_with_dist['s'])):
    if index == 0:
        s_esmini_with_d.append(0)
        s_prev = 0
    else:
        s_diff = adversary_esmini_with_dist['s'][index] - adversary_esmini_with_dist['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_with_d.append(s_current)
        s_prev = s_current

s_esmini_sec_with_d = []
for index in range(len(adversary_esmini_sec_with_dist['s'])):
    if index == 0:
        s_esmini_sec_with_d.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec_with_dist['s'][index] - adversary_esmini_sec_with_dist['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec_with_d.append(s_current+s_cut_diff)
        s_prev = s_current


temp_s_pred = s_esmini_with_d
s_esmini_with_d = []
t_esmini_with_d = []
for index in range(len(temp_s_pred)):
    if (temp_s_pred[index]+s_cut_diff) < 0:
        continue
    else:
        s_esmini_with_d.append(temp_s_pred[index]+s_cut_diff)
        t_esmini_with_d.append(adversary_esmini_with_dist['t'][index]) 

s_cut_esmini_without_d = []
t_cut_esmini_without_d = []
s_cut_esmini_with_d = []
t_cut_esmini_with_d = []
for index in range(len(s_esmini_sec_without_d)):
    if index < cut_time_esmini_without_dist:
        continue
    s_cut_esmini_without_d.append(s_esmini_sec_without_d[index])
    t_cut_esmini_without_d.append(adversary_esmini_sec_without_dist['t'][index])

for index in range(len(s_esmini_sec_with_d)):
    if index < cut_time_esmini_with_dist:
        continue
    s_cut_esmini_with_d.append(s_esmini_sec_with_d[index])
    t_cut_esmini_with_d.append(adversary_esmini_sec_with_dist['t'][index])



s_actual = []
s_pred = []
mse_sec = []
squared_error_s = []
squared_error_t = []
for index in range(len(s_esmini_without_d)):
    if index > 8:
        break
    s_actual = s_cut_esmini_without_d[index]
    s_pred = s_cut_esmini_with_d[index]
    t_actual = t_cut_esmini_without_d[index]
    t_pred = t_cut_esmini_with_d[index]
    squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
    squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
    mse_sec.append(index)

rmse_s_first = math.sqrt(sum(squared_error_s)/len(squared_error_s))
rmse_t_first = math.sqrt(sum(squared_error_t)/len(squared_error_t))

# second noise scenario
s_cut_diff = adversary_esmini_sec_with_dist2['adv_real_esmini_s_diff']
print("s_cut_diff second noise scenario: {}".format(s_cut_diff))

s_esmini_with_d2 = []
for index in range(len(adversary_esmini_with_dist2['s'])):
    if index == 0:
        s_esmini_with_d2.append(0)
        s_prev = 0
    else:
        s_diff = adversary_esmini_with_dist2['s'][index]- adversary_esmini_with_dist2['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_with_d2.append(s_current)
        s_prev = s_current

s_esmini_sec_with_d2 = []
for index in range(len(adversary_esmini_sec_with_dist2['s'])):
    if index == 0:
        s_esmini_sec_with_d2.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec_with_dist2['s'][index] - adversary_esmini_sec_with_dist2['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec_with_d2.append(s_current+s_cut_diff)
        s_prev = s_current


temp_s_pred = s_esmini_with_d2
s_esmini_with_d2 = []
t_esmini_with_d2 = []
for index in range(len(temp_s_pred)):
    if (temp_s_pred[index]+s_cut_diff) < 0:
        continue
    else:
        s_esmini_with_d2.append(temp_s_pred[index]+s_cut_diff)
        t_esmini_with_d2.append(adversary_esmini_with_dist2['t'][index]) 

s_cut_esmini_with_d2 = []
t_cut_esmini_with_d2 = []
for index in range(len(s_esmini_sec_with_d2)):
    if index < cut_time_esmini_with_dist2:
        continue
    s_cut_esmini_with_d2.append(s_esmini_sec_with_d2[index])
    t_cut_esmini_with_d2.append(adversary_esmini_sec_with_dist2['t'][index])



s_actual = []
s_pred = []
mse_sec = []
squared_error_s = []
squared_error_t = []
for index in range(len(s_esmini_without_d)):
    if index > 8:
        break
    s_actual = s_cut_esmini_without_d[index]
    s_pred = s_cut_esmini_with_d2[index]
    t_actual = t_cut_esmini_without_d[index]
    t_pred = t_cut_esmini_with_d2[index]
    squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
    squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
    mse_sec.append(index)

rmse_s_second = math.sqrt(sum(squared_error_s)/len(squared_error_s))
rmse_t_second = math.sqrt(sum(squared_error_t)/len(squared_error_t))



# Third noise scenario
s_cut_diff = adversary_esmini_sec_with_dist3['adv_real_esmini_s_diff']
print("s_cut_diff third noise scenario: {}".format(s_cut_diff))
s_esmini_with_d3 = []
for index in range(len(adversary_esmini_with_dist3['s'])):
    if index == 0:
        s_esmini_with_d3.append(0)
        s_prev = 0
    else:
        s_diff = adversary_esmini_with_dist3['s'][index]-adversary_esmini_with_dist3['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_with_d3.append(s_current)
        s_prev = s_current

s_esmini_sec_with_d3 = []
for index in range(len(adversary_esmini_sec_with_dist3['s'])):
    if index == 0:
        s_esmini_sec_with_d3.append(0+s_cut_diff)
        s_prev = 0
    else:
        s_diff = adversary_esmini_sec_with_dist3['s'][index]- adversary_esmini_sec_with_dist3['s'][index-1]
        s_current = s_prev+s_diff
        s_esmini_sec_with_d3.append(s_current+s_cut_diff)
        s_prev = s_current


temp_s_pred = s_esmini_with_d3
s_esmini_with_d3 = []
t_esmini_with_d3 = []
for index in range(len(temp_s_pred)):
    if (temp_s_pred[index]+s_cut_diff) < 0:
        continue
    else:
        s_esmini_with_d3.append(temp_s_pred[index]+s_cut_diff)
        t_esmini_with_d3.append(adversary_esmini_with_dist3['t'][index]) 

s_cut_esmini_with_d3 = []
t_cut_esmini_with_d3 = []
for index in range(len(s_esmini_sec_with_d3)):
    if index < cut_time_esmini_with_dist3:
        continue
    s_cut_esmini_with_d3.append(s_esmini_sec_with_d3[index])
    t_cut_esmini_with_d3.append(adversary_esmini_sec_with_dist3['t'][index])



s_actual = []
s_pred = []
mse_sec = []
squared_error_s = []
squared_error_t = []
for index in range(len(s_esmini_without_d)):
    if index > 8:
        break
    s_actual = s_cut_esmini_without_d[index]
    s_pred = s_cut_esmini_with_d3[index]
    t_actual = t_cut_esmini_without_d[index]
    t_pred = t_cut_esmini_with_d3[index]
    squared_error_s.append(np.square(np.subtract(s_actual,s_pred)))
    squared_error_t.append(np.square(np.subtract(t_actual,t_pred)))
    mse_sec.append(index)

rmse_s_third = math.sqrt(sum(squared_error_s)/len(squared_error_s))
rmse_t_third = math.sqrt(sum(squared_error_t)/len(squared_error_t))




key = 's'
name = path_to_save_dir+"test_s_t_disturbance"
plt.figure()
plt.figure(figsize=(35,50)) #15 25 #35 60
plt.xlabel("t, lateral displacement", size=100)
plt.ylabel('s, longitudinal displacement',size=100)
plt.xlim([-1.0, -5.0])
plt.ylim([0, 140])
#plt.text(-3, 21, "rmse_s = {:.3f} m".format(rmse_s), fontsize = 100)
#plt.text(-3, 16, "rmse_t = {:.3f} m".format(rmse_t), fontsize = 100)
plt.plot(t_esmini_without_d, s_esmini_without_d, color='r',  linewidth=20)
color = 'm'
plt.plot(t_esmini_with_d, s_esmini_with_d, color='m',  linewidth=20)

plt.plot(t_esmini_with_d2, s_esmini_with_d2, color='c',  linewidth=20)

plt.plot(t_esmini_with_d3, s_esmini_with_d3, color='y',  linewidth=20)

r_size = 95
rt_size = 90

for index in range(len(s_real_sec)): 
    if index >= cut_time_esmini_without_dist-1:
        plt.text(adversary_esmini_sec_without_dist['t'][index],s_esmini_sec_without_d[index]-1.25,str((index+2)-cut_time_esmini_without_dist),fontsize=rt_size,color='w')
        plt.plot(adversary_esmini_sec_without_dist['t'][index]-0.05,s_esmini_sec_without_d[index],marker='o',markersize=r_size,color='r')

for index in range(len(s_real_sec)): 
    if index >= cut_time_esmini_with_dist-1:
        plt.text(adversary_esmini_sec_with_dist['t'][index]+0.1,s_esmini_sec_with_d[index]-1.25,str((index+2)-cut_time_esmini_with_dist),fontsize=rt_size,color='w')
        plt.plot(adversary_esmini_sec_with_dist['t'][index]+0.05,s_esmini_sec_with_d[index],marker='o',markersize=r_size,color='m')



for index in range(len(s_real_sec)): 
    if index >= cut_time_esmini_with_dist2-1:
        plt.text(adversary_esmini_sec_with_dist2['t'][index]+0.05,s_esmini_sec_with_d2[index]-1.25,str((index+2)-cut_time_esmini_with_dist2),fontsize=rt_size,color='b')
        plt.plot(adversary_esmini_sec_with_dist2['t'][index],s_esmini_sec_with_d2[index],marker='o',markersize=r_size,color='c')

for index in range(len(s_real_sec)): 
    if index >= cut_time_esmini_with_dist3-1:
        plt.text(adversary_esmini_sec_with_dist3['t'][index]+0.05,s_esmini_sec_with_d3[index]-1.25,str((index+2)-cut_time_esmini_with_dist3),fontsize=rt_size,color='b')
        plt.plot(adversary_esmini_sec_with_dist3['t'][index],s_esmini_sec_with_d3[index],marker='o',markersize=r_size,color='y')



plt.tick_params(axis='x', labelsize=100)
plt.tick_params(axis='y', labelsize=100)
plt.legend(['Without disturbance', 'Reduced 2 m/s', 'Added 2 m/s', 'Added 4 m/s'], prop={'size':100})
plt.savefig(name)
plt.close()



name = path_to_save_dir+"test_bars"
plt.figure()
plt.figure(figsize=(9,9))
plt.xlabel("scenario", size=25)
plt.ylabel("Longitudinal RMSE", size=25)
rmse = []
rmse.append(rmse_s)
rmse.append(rmse_s_first)
rmse.append(rmse_s_second)
rmse.append(rmse_s_third)
print(rmse)
bars = ('0', '-2 m/s', '+2 m/s', '+ 4 m/s',)
plt.bar(bars, rmse, color=['red','magenta','cyan','yellow'])
plt.savefig(name)
plt.tick_params(axis='x', labelsize=25)
plt.tick_params(axis='y', labelsize=25)
plt.savefig(name)
plt.close()









