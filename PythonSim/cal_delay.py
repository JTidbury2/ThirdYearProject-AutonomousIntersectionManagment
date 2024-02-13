import csv
import sys
from lib.settings import veh_param, cf_param, inter_v_lim, arm_len, veh_dt, simu_t

import numpy as np
import matplotlib.pyplot as plt

def cal_metrics(fname):
    file = open(fname)
    reader = csv.reader(file)
    

    # The columns are start_time, ju_track_len, removed_time, is_removed
    veh_info_table = - np.ones((2000, 4))
    longest_crash_list = []  # Initialize as an empty list

    for i, row in enumerate(reader):
        
        if i == 0 or row[0].startswith('[') or row[0].startswith('p'):
            continue
        if row[0].startswith('update_title_pos'):
            break
        if row[0].startswith("Vehicle") or row[0].startswith("Faulty"):
            continue
        if row[0].startswith("crashed_Vehicle_ID"):
                
                # Initialize an empty list to hold vehicle IDs for this row
                crash_list = []
                
                # Loop through each element in the row
                for element in row:
                    if element.startswith("crashed_Vehicle_ID"):
                        # For the first element, split at ':' and take the part after it, strip '[' and convert to int
                        first_id = element.split(':')[1].strip('[').strip(']')
                        if first_id == '' :
                            continue
                        crash_list.append(int(first_id))
                    else:
                        # For subsequent elements, simply strip whitespace and convert to int
                        crash_list.append(int(element.strip().strip('[').strip(']')))

                # Update longest_crash_list if the current list is longer
                if len(crash_list) > len(longest_crash_list):
                    longest_crash_list = crash_list

                continue  # Continue to the next row

        t, veh_id, zone, x = int(row[0]), int(row[1]), row[2].strip(), float(row[4])
        if t >= simu_t / veh_dt:
            break
        if zone == 'ap':
            if veh_info_table[veh_id, 0] == -1:
                veh_info_table[veh_id, 0] = t
        if zone == 'ju':
            veh_info_table[veh_id, 1] = max(veh_info_table[veh_id, 1], x)
        if zone == 'ex':
            veh_info_table[veh_id, 2] = max(veh_info_table[veh_id, 2], t)
            if x >= arm_len + (veh_param['veh_len'] - veh_param['veh_len_front']):
                veh_info_table[veh_id, 3] = 1

    metrics = {}

    # Find the smallest vehicle that has not completed the entire journey
    veh_not_finish_min = np.where(veh_info_table[:, 3] < 0)[0][0] # The smallest vehicle that has not completed the entire journey
    metrics['veh_not_finish_min'] = veh_not_finish_min
    veh_info_table = veh_info_table[0: veh_not_finish_min, :]

    # Calculate the actual traffic capacity (the flow rate from the first vehicle leaving to the last vehicle leaving the intersection)
    veh_finish_count = np.sum(veh_info_table[:, 3]) # Number of vehicles that have completed the entire journey
    earlist_finish_time = np.min(veh_info_table[:, 2]) # The earliest time to complete the entire process
    actual_total_flow = veh_finish_count / ((t - earlist_finish_time) * veh_dt) * 3600
    metrics['actual_total_flow'] = actual_total_flow

    # Calculate delays
    # real time
    actual_time = (veh_info_table[:, 2] - veh_info_table[:, 0]) * veh_dt
    #Ideal passing time, ignore intersections and other vehicles, and pass at a constant speed
    ideal_time = (arm_len * 2 + veh_info_table[:, 1]) / cf_param['v0']
    delay = actual_time - ideal_time
    metrics['avg_delay'] = np.mean(delay)
    metrics['max_delay'] = np.max(delay)
    metrics['longest_crash_list'] = longest_crash_list  

    plt.plot(delay)
    plt.xlabel('Vehicle Id')
    plt.ylabel('Delay / s')
    plt.grid(True)
    plt.savefig(fname[:-4]+'.png')

    return metrics

def see_veh_avx(fname, id):
    file = open(fname)
    reader = csv.reader(file)
    x_ap = []
    t_ap = []
    x_ju = []
    t_ju = []
    x_ex = []
    t_ex = []
    for i, row in enumerate(reader):
        if i == 0:
            continue
        t, veh_id, zone, x = int(row[0]), int(row[1]), row[2].strip(), float(row[4])
        if id != veh_id:
            continue
        if zone == 'ap':
            t_ap.append(t * veh_dt)
            x_ap.append(x)
        elif zone == 'ju':
            t_ju.append(t * veh_dt)
            x_ju.append(x)
        else:
            t_ex.append(t * veh_dt)
            x_ex.append(x)
    
    v_ap = [(x_ap[i+1] - x_ap[i])/veh_dt for i in range(len(x_ap)-1)]
    a_ap = [(v_ap[i+1] - v_ap[i])/veh_dt for i in range(len(v_ap)-1)]
    v_ju = [(x_ju[i+1] - x_ju[i]) for i in range(len(x_ju)-1)]
    a_ju = [(v_ju[i+1] - v_ju[i]) for i in range(len(v_ju)-1)]
    plt.subplot(321)
    plt.plot(t_ap, x_ap)
    plt.subplot(322)
    plt.plot(t_ju, x_ju)
    plt.subplot(323)
    plt.plot(t_ap[:-1], v_ap)
    plt.subplot(324)
    plt.plot(t_ju[:-1], v_ju)
    plt.subplot(325)
    plt.plot(t_ap[:-2], a_ap)
    plt.subplot(326)
    plt.plot(t_ju[:-2], a_ju)
    plt.show()

if __name__ == '__main__':

    files = [
        'log 2019-05-28 10-56-52.log'
    ]

    for file in files:
        metrics = cal_metrics('log/' + file)
        print(file)
        for key, value in metrics.items():
            print(key, '=', value)
        print('')

