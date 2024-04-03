import csv
import sys
from lib.settings import veh_param, cf_param, inter_v_lim, arm_len, veh_dt, simu_t, liveValues, random_veh_param,arm_v_lim
import math
import numpy as np
import matplotlib.pyplot as plt

offsetArray = [[0.34,0.2354,1.4054,0.05],[0.235,0.24,0.34,-0.056],[0.35,0.25,0.35,0.08],[0,0,0,0]]

def cal_metrics(fname):
    file = open(fname)
    reader = csv.reader(file)
    

    # The columns are start_time, ju_track_len, removed_time, is_removed
    veh_info_table = - np.ones((2000, 6))
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

        t, veh_id, zone, x, turn_dir, type = int(row[0]), int(row[1]), row[2].strip(), float(row[4]), row[7].strip(), int(row[8])
        offset2 = 2.5 if liveValues["random"] else 0
        if t >= simu_t / veh_dt:
            break
        if zone == 'ap':
            if veh_info_table[veh_id, 0] == -1:
                veh_info_table[veh_id, 0] = t
        if zone == 'ju':
            veh_info_table[veh_id, 1] = max(veh_info_table[veh_id, 1], x)
        if zone == 'ex':
            veh_info_table[veh_id, 2] = max(veh_info_table[veh_id, 2], t)
            if x >= arm_len -offset2+ (veh_param['veh_len'] - veh_param['veh_len_front']):
                veh_info_table[veh_id, 3] = 1
                veh_info_table[veh_id, 4] = turn_dir
                veh_info_table[veh_id, 5] = type


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
    # print("veh_info_table", veh_info_table)
    # real time
    actual_time = (veh_info_table[:, 2] - veh_info_table[:, 0]) * veh_dt
    #Ideal passing time, ignore intersections and other vehicles, and pass at a constant speed
    # print("veh_info_table[:, 5]", veh_info_table[:, 5])
    total_distance = arm_len * 2 + veh_info_table[:, 1]


    # print("total_distance", total_distance)
    ideal_time = []
    for index,value in enumerate(total_distance):
        average_max_v = min(random_veh_param[int(veh_info_table[index][5])]["max_v"], arm_v_lim)
        avearge_max_acc = random_veh_param[int(veh_info_table[index][5])]["max_acc"]
        if(value<(average_max_v**2-cf_param["v0"]**2)/(2*avearge_max_acc)):
            # print("The total distance is too short, the vehicle will not reach the maximum speed.")
            # Coefficients for the quadratic equation
            a = 0.5 * avearge_max_acc
            b = cf_param["v0"]
            c = -value
            
            # Calculate the discriminant
            discriminant = b**2 - 4*a*c
            
            # Assuming discriminant is positive, so we have real solutions
            if discriminant >= 0:
                # Only the positive root is physically meaningful
                ideal_time.append(((-b + math.sqrt(discriminant)) / (2 * a))+offsetArray[int(veh_info_table[index][4])][int(veh_info_table[index][5])])
                # print(f"The ideal time to cover the distance is {ideal_time} seconds.")
            else:
                # print("No real solution exists. Check the input parameters.")
                pass
        else:
            # print("The total distance is long enough for the vehicle to reach the maximum speed.")
                # Step 1: Calculate distance needed to reach max speed
            distance_to_max_speed = (average_max_v**2 - cf_param["v0"]**2) / (2 * avearge_max_acc)
            
            # Step 2: Calculate time to reach max speed
            time_to_max_speed = (average_max_v - cf_param["v0"]) / avearge_max_acc
            
            # Step 3: Calculate remaining distance to be covered at max speed
            remaining_distance = value - distance_to_max_speed
            
            # Step 4: Calculate time to cover the remaining distance at max speed
            time_at_max_speed = remaining_distance / average_max_v
            
            # Step 5: Sum up the times
            ideal_time.append(time_to_max_speed + time_at_max_speed+offsetArray[int(veh_info_table[index][4])][int(veh_info_table[index][5])])

        


    # ideal_time = (arm_len * 2 + veh_info_table[:, 1]) / cf_param['v0']
    # print("ideal_time", ideal_time)
    # print("actual_time", actual_time)
    # print("type/way pairng", [(int(veh_info_table[index][4]),int(veh_info_table[index][5])) for index in range(len(veh_info_table))])

    delay = actual_time - ideal_time #round of stuff that i found
    delay = [max(0, x) for x in delay]
    # print("Delay", delay)
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

