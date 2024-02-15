import math
import logging

from lib.settings import inter_control_mode, lane_width, turn_radius, arm_len, NS_lane_count, EW_lane_count, veh_dt, inter_v_lim, inter_v_lim_min, min_gen_ht, conflict_movements, virtual_lead_v, desired_cf_distance, phase, yellow_time,crashValues
from map import Map, Track

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

class BaseInterManager:
    def __init__(self):
        self.timestep = 0
    def update(self):
        self.timestep += 1
    def receive_V2I(self, sender, message):
        pass

        
    def check_for_collision(self,all_vehicles):
        pass

class TrafficLightManager(BaseInterManager):
    def __init__(self):
        super().__init__()
        self.current_phase = 0
        self.current_elapsed_time = 0
        self.phase = phase


    def update(self):
        super().update()
        self.update_phase()
    
    def update_phase(self):
        self.current_elapsed_time += 1
        message = {}
        if self.current_elapsed_time == self.phase[self.current_phase][0]:
            # This phase is over, change to the next phase
            self.current_elapsed_time = 0
            self.current_phase = (self.current_phase + 1) % len(self.phase)
        if self.current_elapsed_time >= self.phase[self.current_phase][0] - (yellow_time / veh_dt):
            # It's yellow light time
            for ap_arm_dir in ['Nl', 'Nt', 'Nr', 'Sl', 'St', 'Sr', 'El', 'Et', 'Er', 'Wl', 'Wt', 'Wr']:
                if ap_arm_dir in self.phase[self.current_phase]:
                    message[ap_arm_dir] = 'Y'
                else: 
                    message[ap_arm_dir] = 'R'
            ComSystem.I_broadcast(message)
            # print('Yellow light = [%s]' % str(self.phase[self.current_phase][1:]))
        else:
            for ap_arm_dir in ['Nl', 'Nt', 'Nr', 'Sl', 'St', 'Sr', 'El', 'Et', 'Er', 'Wl', 'Wt', 'Wr']:
                if ap_arm_dir in self.phase[self.current_phase]:
                    message[ap_arm_dir] = 'G'
                else: 
                    message[ap_arm_dir] = 'R'
            ComSystem.I_broadcast(message)
            # print('Green light = [%s]' % str(self.phase[self.current_phase][1:]))
        
    def receive_V2I(self, sender, message):
        return 
    


class DresnerManager(BaseInterManager):
    def __init__(self):
        super().__init__()
        self.res_grid = DresnerResGrid(0.5) # Write to settings?
        self.running_grid = DresnerResGrid(0.1)
        self.ex_lane_table = self.gen_ex_lane_table()
        self.res_registery = {}
        self.crash_happened = False

    def update(self):
        super().update()
        self.res_grid.dispose_passed_time(self.timestep)

    def get_grid_cells(self):
        return self.running_grid.cells
    
    def get_grid_location(self,veh):
        seg_idx = 0
        for (i, end_x) in enumerate(veh.track.ju_shape_end_x):
            if veh.inst_x > end_x: # is greater than the end point of the i-th segment, then it is in the (i+1) segment
                seg_idx = i + 1
                break
        seg = veh.track.ju_track[seg_idx] #The shape of this segment
        if seg_idx > 0:
            seg_x = veh.inst_x - veh.track.ju_shape_end_x[seg_idx - 1] # The length of this segment
        else:
            seg_x = veh.inst_x
        if seg[0] == 'line': # is a straight line
            if abs(seg[1][0] - seg[2][0]) < 1e-5: # vertical bar
                x = seg[1][0]
                if seg[1][1] < seg[2][1]: # from top to bottom
                    y = seg[1][1] + seg_x
                    angle = 180 # angle is the number of degrees of clockwise rotation compared to "head to north"
                else: # from bottom to top
                    y = seg[1][1] - seg_x
                    angle = 0
            else: # Horizontal line
                y = seg[1][1]
                if seg[1][0] < seg[2][0]: # from left to right
                    x = seg[1][0] + seg_x
                    angle = 90 
                else: # from right to left
                    x = seg[1][0] - seg_x
                    angle = 270
        else:  # circular curve
            if seg[5][0] < seg[5][1]: # Trajectory counterclockwise
                rotation = seg[5][0] + seg_x / seg[4] * 180 / math.pi
                angle = 180 - rotation
                x = seg[3][0] + seg[4] * math.cos(-rotation / 180 * math.pi)
                y = seg[3][1] + seg[4] * math.sin(-rotation / 180 * math.pi)
            else:
                rotation = seg[5][0] - seg_x / seg[4] * 180 / math.pi
                angle = - rotation
                x = seg[3][0] + seg[4] * math.cos(-rotation / 180 * math.pi)
                y = seg[3][1] + seg[4] * math.sin(-rotation / 180 * math.pi)

        # Calculate the xy coordinates of the vehicle's dots in the logical coordinate system (first rotate, then place in xy)
        veh_dots_x, veh_dots_y = self.gen_veh_dots(veh.veh_wid, veh.veh_len, veh.veh_len_front, \
            0.4, veh.inst_v * 0.1)
        veh_dots_x_rt = veh_dots_x * math.cos(angle*math.pi/180) - veh_dots_y * math.sin(angle*math.pi/180)
        veh_dots_y_rt = veh_dots_y * math.cos(angle*math.pi/180) + veh_dots_x * math.sin(angle*math.pi/180)
        veh_dots_x_rt += x
        veh_dots_y_rt += y
        i, j = self.running_grid.xy_to_ij(veh_dots_x_rt, veh_dots_y_rt)

        return i, j 
    
    def check_for_collision(self,all_vehicles):
        crashed_Vehicle_ID=[]
        if self.crash_happened:
            for veh in all_vehicles:
                i,j = self.get_grid_location(veh)

                for idx in range(len(i)):
                    current_cell = self.get_grid_cells()[i[idx], j[idx], 0]  # Access the current cell
                    
                    if current_cell != -1 and current_cell != veh._id:  # If the cell is occupied by another vehicle
                        if current_cell not in crashed_Vehicle_ID:  # If the occupying vehicle is not already in the list
                            crashed_Vehicle_ID.append(current_cell)  # Add the occupying vehicle ID to the list

                        if veh._id not in crashed_Vehicle_ID:  # If the current vehicle is not already in the list
                            crashed_Vehicle_ID.append(veh._id)  # Add the current vehicle ID to the list

                        reply_message = {'type': 'collision'}
                        ComSystem.I2V(veh, reply_message)  # Send a collision message to the current vehicle

                        # Assuming you have a way to send messages to other vehicles by ID
                        ComSystem.I2V(self.get_vehicle_by_id(current_cell,all_vehicles), reply_message)  # Send a collision message to the occupying vehicle

                    # Update the cell to indicate it's now occupied by the current vehicle
                    self.get_grid_cells()[i[idx], j[idx], 0] = veh._id
            self.running_grid.reset_grid()

        logging.debug('crashed_Vehicle_ID:%s', crashed_Vehicle_ID)
        return crashed_Vehicle_ID
    
    def check_for_collision_noCars(self):
        return self.crash_happened
    
    def get_vehicle_by_id(self,veh_id,all_vehicles):
        for veh in all_vehicles:
            if veh._id == veh_id:
                return veh
        return None
            

        
        

    def receive_V2I(self, sender, message):
        if message['type'] == 'request':
            if self.crash_happened:
                reply_message = {
                    'type': 'reject',
                    'timeout': 1
                }
                ComSystem.I2V(sender, reply_message)
            reservation = self.check_request(message)
            if reservation:
                reply_message = {
                    'type': 'confirm',
                    'reservation': reservation
                }
                self.res_registery[message['veh_id']] = reservation['res_id']
                ComSystem.I2V(sender, reply_message)
            else: 
                reply_message = {
                    'type': 'reject',
                    'timeout': 1
                }
                ComSystem.I2V(sender, reply_message)
        elif message['type'] == 'change-request':
            pass
        elif message['type'] == 'cancel':
            # process cancel with P
            pass
        elif message['type'] == 'done':
            # record any statistics supplied in message
            # process cancel with P
            self.res_registery.pop(message['veh_id']) # dict.pop(key)返回value并删除
            ComSystem.I2V(sender, {
                'type': 'acknowledge',
                'res_id': message['res_id']
            })
        elif message["type"]== "fault":
            self.crash_occured()

    def crash_occured(self):
        self.crash_happened=True
        crashValues['crashOccured']=True
        ComSystem.I_broadcast({'type': 'crash'})

    

    def gen_ex_lane_table(self):
        table = {}
        table['Nl'] = list(range(EW_lane_count))
        table['Sl'] = table['Nl']
        table['El'] = list(range(NS_lane_count))
        table['Wl'] = table['El']
        table['Nr'] = list(range(EW_lane_count - 1, -1, -1))
        table['Sr'] = table['Nr']
        table['Er'] = list(range(NS_lane_count - 1, -1, -1))
        table['Wr'] = table['Sr']
        for i in range(NS_lane_count):
            ti_list = [i]
            left_finished, right_finished = False, False
            for j in range(1, EW_lane_count):
                if not left_finished:
                    if i - j >= 0:
                        ti_list.append(i - j)
                    else:
                        left_finished = True
                if not right_finished:
                    if i + j < EW_lane_count:
                        ti_list.append(i + j)
                    else:
                        right_finished = True
                if left_finished and right_finished:
                    break
            table['Nt' + str(i)] = ti_list
            table['St' + str(i)] = ti_list
        for i in range(EW_lane_count):
            ti_list = [i]
            left_finished, right_finished = False, False
            for j in range(1, NS_lane_count):
                if not left_finished:
                    if i - j >= 0:
                        ti_list.append(i - j)
                    else:
                        left_finished = True
                if not right_finished:
                    if i + j < NS_lane_count:
                        ti_list.append(i + j)
                    else:
                        right_finished = True
                if left_finished and right_finished:
                    break
            table['Et' + str(i)] = ti_list
            table['Wt' + str(i)] = ti_list
        return table

    def get_ex_lane_list(self, ap_arm, turn_dir, ap_lane):
        if turn_dir == 't':
            return self.ex_lane_table[ap_arm + turn_dir + str(ap_lane)]
        else:
            return self.ex_lane_table[ap_arm + turn_dir]

    def gen_veh_dots(self, veh_wid, veh_len, veh_len_front, static_buf, time_buf):
        '''Cut the vehicles into sufficiently dense scattered points. The front of the vehicle faces north, xy is the same as the drawing logical coordinate system, x is to the right, and y is downward. The center of the vehicle's front wheels is at (0,0). Returns the x and y of the retrieved series of points. '''
        xs = np.arange(-veh_wid/2 - static_buf, veh_wid/2 + static_buf + 1e-1, 0.2)
        ys = np.arange(-veh_len_front - static_buf, veh_len - veh_len_front + 1e-1 + static_buf, 0.2)
        xx, yy = np.meshgrid(xs, ys)
        if time_buf > static_buf:
            xs_tb = np.arange(-veh_wid/2, veh_wid/2 + 1e-1, 0.2) # 车宽
            ys_tb_front = np.arange(-veh_len_front - time_buf, -veh_len_front - static_buf + 1e-1, 0.2)
            ys_tb_back = np.arange(veh_len - veh_len_front + static_buf, veh_len - veh_len_front + time_buf + 1e-1, 0.2)
            ys_tb = np.append(ys_tb_front, ys_tb_back)
            xx_tb, yy_tb = np.meshgrid(xs_tb, ys_tb)
            xx = np.append(xx, xx_tb)
            yy = np.append(yy, yy_tb)
        return [xx.flatten(), yy.flatten()]

    def check_request(self, message): 
        ex_arm = Map.getInstance().get_ex_arm(message['arr_arm'], message['turn_dir'])
        ex_lane_list = self.get_ex_lane_list(message['arr_arm'], message['turn_dir'], message['arr_lane'])
        for ex_lane in ex_lane_list:
            ju_track = Map.getInstance().get_ju_track(message['arr_arm'], message['turn_dir'], message['arr_lane'], ex_lane)
            ju_shape_end_x = Track.cal_ju_shape_end_x(ju_track)
            acc_distance = (inter_v_lim**2 - message['arr_v']**2) / 2 / message['max_acc']
            exit_time = message['arr_t']  # Initialize exit time with the arrival time

            if acc_distance >= ju_shape_end_x[-1]: 
                # Accelerate the whole process
                acc_acc = [[message['arr_t'], message['max_acc']]]
                # Estimate exit time assuming constant acceleration over the distance
                exit_time += ((2 * ju_shape_end_x[-1]) / message['max_acc']) ** 0.5
            else:
                # Acceleration-constant speed
                acc_time = (inter_v_lim - message['arr_v']) / message['max_acc']
                constant_speed_distance = ju_shape_end_x[-1] - acc_distance
                constant_speed_time = constant_speed_distance / inter_v_lim
                exit_time += acc_time + constant_speed_time

                acc_acc = [
                    [message['arr_t'], message['max_acc']], 
                    [message['arr_t'] + acc_time, 0]
                ]

            if message['arr_v'] < inter_v_lim_min:
                acc_distance_c = (8**2 - message['arr_v']**2) / 2 / message['max_acc']
                if acc_distance_c >= ju_shape_end_x[-1]: 
                    acc_const_v = [[message['arr_t'], message['max_acc']]]
                    exit_time += ((2 * ju_shape_end_x[-1]) / message['max_acc']) ** 0.5
                else:
                    acc_time_c = (8 - message['arr_v']) / message['max_acc']
                    constant_speed_distance_c = ju_shape_end_x[-1] - acc_distance_c
                    constant_speed_time_c = constant_speed_distance_c / 8
                    exit_time += acc_time_c + constant_speed_time_c

                    acc_const_v = [
                        [message['arr_t'], message['max_acc']], 
                        [message['arr_t'] + acc_time_c, 0]
                    ]
            else:
                acc_const_v = [[message['arr_t'], 0]]
                constant_speed_distance = ju_shape_end_x[-1]
                constant_speed_time = constant_speed_distance / message['arr_v']
                exit_time += constant_speed_time

            if self.check_cells_stepwise(message, ju_track, ju_shape_end_x, ex_arm, ex_lane, acc_acc):
                return {
                    'res_id': 0,  # Todo: Generate a unique reservation ID
                    'ex_lane': ex_lane,
                    'arr_t': message['arr_t'],
                    'arr_v': message['arr_v'],
                    'acc': acc_acc,
                    'exit_time': exit_time  # Include the calculated exit time
                }
            elif self.check_cells_stepwise(message, ju_track, ju_shape_end_x, ex_arm, ex_lane, acc_const_v):
                return {
                    'res_id': 0,  # Todo: Generate a unique reservation ID
                    'ex_lane': ex_lane,
                    'arr_t': message['arr_t'],
                    'arr_v': message['arr_v'],
                    'acc': acc_const_v,
                    'exit_time': exit_time  # Include the calculated exit time
                }
        return None

            
    def check_cells_stepwise(self, message, ju_track, ju_shape_end_x, ex_arm, ex_lane, acc):
        t = message['arr_t'] #Currtent time
        v = message['arr_v'] #Current speed
        x_1d = 0 # One dimensional positon along junciton path
        a_idx = 0 # Acceleration index
        seg_idx = 0 # Segment index

        while x_1d <= ju_shape_end_x[-1]:
            # Calculate the vehicle xy coordinates and direction
            seg = ju_track[seg_idx]
            if seg_idx > 0:
                seg_x = x_1d - ju_shape_end_x[seg_idx - 1]  
            else:
                seg_x = x_1d
            if seg[0] == 'line': # is a straight line
                if abs(seg[1][0] - seg[2][0]) < 1e-5: # vertical bar
                    x = seg[1][0]
                    if seg[1][1] < seg[2][1]: # from top to bottom
                        y = seg[1][1] + seg_x
                        angle = 180 # angle is the number of degrees of clockwise rotation compared to "head to north"
                    else: # from bottom to top
                        y = seg[1][1] - seg_x
                        angle = 0
                else: # Horizontal line
                    y = seg[1][1]
                    if seg[1][0] < seg[2][0]: # from left to right
                        x = seg[1][0] + seg_x
                        angle = 90 
                    else: # from right to left
                        x = seg[1][0] - seg_x
                        angle = 270
            else:  # circular curve
                if seg[5][0] < seg[5][1]: # Trajectory counterclockwise
                    rotation = seg[5][0] + seg_x / seg[4] * 180 / math.pi
                    angle = 180 - rotation
                    x = seg[3][0] + seg[4] * math.cos(-rotation / 180 * math.pi)
                    y = seg[3][1] + seg[4] * math.sin(-rotation / 180 * math.pi)
                else:
                    rotation = seg[5][0] - seg_x / seg[4] * 180 / math.pi
                    angle = - rotation
                    x = seg[3][0] + seg[4] * math.cos(-rotation / 180 * math.pi)
                    y = seg[3][1] + seg[4] * math.sin(-rotation / 180 * math.pi)
            
            # Calculate the xy coordinates of the vehicle's dots in the logical coordinate system (first rotate, then place in xy)
            veh_dots_x, veh_dots_y = self.gen_veh_dots(message['veh_wid'], message['veh_len'], message['veh_len_front'], \
                0.4, v * 0.1)
            veh_dots_x_rt = veh_dots_x * math.cos(angle*math.pi/180) - veh_dots_y * math.sin(angle*math.pi/180)
            veh_dots_y_rt = veh_dots_y * math.cos(angle*math.pi/180) + veh_dots_x * math.sin(angle*math.pi/180)
            veh_dots_x_rt += x
            veh_dots_y_rt += y
            
            #Use grid.xy_to_ij to convert into the cell occupied by the vehicle at this time
            i, j = self.res_grid.xy_to_ij(veh_dots_x_rt, veh_dots_y_rt)
            t_slice = np.ones(i.shape, dtype=np.int16) * (round(t-self.res_grid.t_start))
            while t_slice[0] >= self.res_grid.cells.shape[2]:
                self.res_grid.add_time_dimension()
            
            # Check whether all occupied grid points are empty
            if np.sum(self.res_grid.cells[i, j, t_slice] != -1) == 0:
                self.res_grid.cells[i, j, t_slice] = message['veh_id']
                if message['veh_id']==1:
                    print('i:',i)
                    print('j:',j)
                    print('t_slice:',t_slice)
            else:
                # Planning failed, return False after clearing traces
                self.res_grid.clear_veh_cell(message['veh_id'])
                return False

            # Update position, velocity, acceleration, and shape
            x_1d += v * veh_dt + acc[a_idx][1] / 2 * veh_dt ** 2
            v += acc[a_idx][1] * veh_dt
            t += 1
            if a_idx+1 < len(acc) and t >= acc[a_idx+1][0]:
                a_idx += 1
            if x_1d > ju_shape_end_x[seg_idx]:
                seg_idx += 1 # If it is the last shape, it will exit the loop, it’s okay

        occ_dura = max((v-inter_v_lim_min)/message['max_dec'] + message['veh_len']/v, min_gen_ht)
        occ_start = math.floor(t - (occ_dura / veh_dt))
        occ_end = math.ceil(t)
        for record in self.res_grid.ex_lane_record[ex_arm + str(ex_lane)]:
            if not (record[1] > occ_end or record[2] < occ_start):
                self.res_grid.clear_veh_cell(message['veh_id'])
                return False

        self.res_grid.ex_lane_record[ex_arm + str(ex_lane)].append([message['veh_id'], occ_start, occ_end])
        return True
    
class GeneticReordering():
    def __init__(self, default_processing_interval_arg=2.0):
        # The time period between the processing times.
        self.default_processing_interval = 2.0
        # The amount of lookahead (the period of time between the end time of the
        # last processed batch and the target batch).
        self.lookahead_time=3.0

        self.batch_interval=self.default_processing_interval
        self.next_processing_time=0.0
        self.next_proposal_deadline=0.0
        self.processing_interval=default_processing_interval_arg

        def setInitialTime(self, initTime):
            self.next_processing_time=initTime+self.processing_interval
            self.next_proposal_deadline=self.next_processing_time
        
        def get_batch(current_time, queue):
            # Assuming selectProposals and reorderProposals are defined elsewhere and work similarly to the Java version
            proposals1 = self.select_proposals(current_time, queue)
            proposals2 = self.reorder_proposals(proposals1)

            global next_processing_time, next_proposal_deadline
            self.next_processing_time = current_time + self.processing_interval
            self.next_proposal_deadline = next_processing_time

            return proposals2

        def select_proposals(self, current_time, queue):
            result = []

            start_time = current_time + self.lookahead_time
            end_time = start_time + self.batch_interval

            for request in queue:
                arrival_time = request['arr_t']  # Assuming get_arrival_time method exists in Proposal class

                if arrival_time < end_time:
                    result.append(request)
                else:
                    break

            return result

        def reorder_proposals(self, proposals):
                # A partition of the proposals according to the road of the arrival lane.
                partition = {}

                for proposal in proposals:
                    lane_id = proposal['arr_lane']  # Assuming this is equivalent to getArrivalLaneID

                    if lane_id in partition:
                        partition[lane_id].append(proposal)
                    else:
                        partition[lane_id] = [proposal]

                # Combine the proposals according to the partition
                result = []
                for road_proposals in partition.values():
                    result.extend(road_proposals)

                return result


    
class GeneticManager(DresnerManager):
    def __init__(self):
        super().__init__()
        self.reordering_strategy = GeneticReordering()
        self.queue = []
        self.next_processing_time = 0.0
        self.next_proposal_deadline = 0.0
        self.processing_interval = 2.0
        self.last_vin_in_batch = set()

    
    def receive_V2I(self, sender, message):
        if message['type'] == 'request':
            if self.crash_happened:
                reply_message = {
                    'type': 'reject',
                    'timeout': 1
                }
                ComSystem.I2V(sender, reply_message)
            reservation = self.check_request(message)
            if reservation:
                reply_message = {
                    'type': 'confirm',
                    'reservation': reservation
                }
                self.res_registery[message['veh_id']] = reservation['res_id']
                ComSystem.I2V(sender, reply_message)
            else: 
                reply_message = {
                    'type': 'reject',
                    'timeout': 1
                }
                ComSystem.I2V(sender, reply_message)
        elif message['type'] == 'change-request':
            pass
        elif message['type'] == 'cancel':
            # process cancel with P
            pass
        elif message['type'] == 'done':
            # record any statistics supplied in message
            # process cancel with P
            self.res_registery.pop(message['veh_id']) # dict.pop(key)返回value并删除
            ComSystem.I2V(sender, {
                'type': 'acknowledge',
                'res_id': message['res_id']
            })
        elif message["type"]== "fault":
            self.crash_occured()

    def geneticAct(self,timeStep):#
        if self.base_policy.get_current_time() >= self.next_processing_time:
            vin_in_batch = self.process_batch()
            
            # if IS_HIGHLIGHT_VEHICLE_IN_BATCH:
            #     for vin in self.last_vin_in_batch:
            #         Debug.remove_vehicle_color(vin)
            #     for vin in vin_in_batch:
            #         Debug.set_vehicle_color(vin, VEHICLE_IN_BATCH_COLOR)
                
                # self.last_vin_in_batch = vin_in_batch
            
            self.next_processing_time = self.reordering_strategy.get_next_processing_time()
            self.next_proposal_deadline = self.reordering_strategy.get_next_proposal_deadline()

            # After updating the proposal deadline, immediately confirm/reject
            # the indexed proposal in the queue whose expiration time is before
            # the new proposal deadline.
            self.try_reserve_for_proposals_before_time(self.next_proposal_deadline)

    def process_batch(self):
        vin_in_batch = set()

        current_time = self.base_policy.get_current_time()

        # Ensure that no proposal in the queue is before the deadline
        # This is a direct translation of the assert statement in Java
        assert len(self.queue) == 0 or self.queue[0].get_proposal().get_arrival_time() >= self.next_proposal_deadline

        # Retrieve the batch (the set of indexed proposals)
        batch = self.reordering_strategy.get_batch(current_time, self.queue)

        # Confirm or reject the proposals in the batch according to the new ordering
        for i_proposal in batch:
            self.try_reserve(i_proposal)
            vin_in_batch.add(i_proposal.get_request().get_vin())

        return vin_in_batch
    

    def try_reserve(self, i_proposal):
        l = [i_proposal.get_proposal()]  # Create a list with the proposal
        msg = i_proposal.get_request()
        reserve_param = self.base_policy.find_reserve_param(msg, l)
        
        if reserve_param is not None:
            self.base_policy.send_confirm_msg(msg['request_id'], reserve_param)
            # Remove a set of indexed proposals (including the given one) from the queue
            for i_proposal2 in i_proposal.get_proposal_group():
                self.queue.remove(i_proposal2)  # Assuming queue supports direct removal
        else:
            # Remove the indexed proposal from the queue
            self.queue.remove(i_proposal)
            # Shrink the proposal group
            ip_group = i_proposal.get_proposal_group()
            if i_proposal in ip_group:
                ip_group.remove(i_proposal)
                # If the proposal group is empty, no proposal left for the request message
                # and need to send the reject message
                if not ip_group:
                    self.base_policy.send_reject_msg(msg['vin'],
                                                     msg['request_id'],
                                                     'NO_CLEAR_PATH')  # Assuming 'NO_CLEAR_PATH' is a constant or enum value
            else:  # The removal is unsuccessful
                raise RuntimeError("BatchModeRequestHandler: Proposal Group error: unable to remove an indexed proposal.")
            
    def process_request_msg(self, msg):
        vin = msg['veh_id']


        # # If the vehicle has got a reservation already, reject it.
        # if self.base_policy.has_reservation(vin):
        #     self.base_policy.send_reject_msg(vin,
        #                                      msg['request_id'],
        #                                      'CONFIRMED_ANOTHER_REQUEST')  # Assuming this is a constant or an enum value
        #     if self.requestSC is not None:
        #         self.requestSC.incr_num_of_confirmed_another_request()
        #     return

        # First, remove the proposals of the vehicle (if any) in the queue.
        self.remove_request_by_veh_id(vin)

        current_time = self.base_policy.get_current_time()
        proposals = msg['proposals']  # Assuming this key exists and contains a list of proposals

        # Filter the proposals
        filter_result = self.base_policy.standard_proposals_filter(proposals, current_time)

        if filter_result.is_no_proposal_left():
            # Reject immediately since the existing proposals of the vehicle have been removed from the queue.
            self.base_policy.send_reject_msg(vin,
                                             msg['request_id'],
                                             filter_result.get_reason())
            return

        if self.is_all_proposals_late(msg):
            # Immediately confirm/reject the remaining proposals.
            reserve_param = self.base_policy.find_reserve_param(msg, filter_result.get_proposals())
            if reserve_param is not None:
                self.base_policy.send_confirm_msg(msg['request_id'], reserve_param)
            else:
                self.base_policy.send_reject_msg(vin, msg['request_id'], 'NO_CLEAR_PATH')
            if self.requestSC is not None:
                self.requestSC.incr_num_of_late_request()
        else:
            # Put the proposals in the queue and postpone the processing of these proposals.
            self.put_proposals_into_queue(msg, current_time)
            if self.requestSC is not None:
                self.requestSC.incr_num_of_queued_request()

    def remove_request_by_veh_id(self, vin):
            self.queue = [request for request in self.queue if request['veh_id'] != vin]
    















class DresnerResGrid:
    '''a grid representation of intersection area'''
    def __init__(self, cell_size):
        self.lw = lane_width
        self.tr = turn_radius
        self.al = arm_len
        self.NSl = NS_lane_count
        self.EWl = EW_lane_count
        # Half the intersection width and height, in m, that is (x2, y2)
        self.wid_m_half = self.lw * self.NSl + self.tr
        self.hgt_m_half = self.lw * self.EWl + self.tr
        self.cell_size = cell_size
        #The number of rows and columns of grid
        self.i_n = math.ceil(self.hgt_m_half * 2 / cell_size) # Number of rows
        self.j_n = math.ceil(self.wid_m_half * 2 / cell_size) #Number of columns

        self.t_start = 0 # The timestep corresponding to the third dimension t=0

        self.cells = - np.ones(shape=(self.i_n, self.j_n, int(20/veh_dt)), dtype=np.int16)
        self.ex_lane_record = self.init_ex_lane_record() # This is to avoid collision at the exit lane. No car can arrive within a certain period of time before each car arrives.

    def reset_grid(self):
        self.cells = - np.ones(shape=(self.i_n, self.j_n, int(20/veh_dt)), dtype=np.int16)

    def xy_to_ij(self, x_arr, y_arr):
        '''
        Find the corresponding cell index ij from the xy coordinates. xyij can all be vectors.
        The xy coordinate is the logical coordinate system in my_paint_canvas. The right and bottom are the positive x and y directions respectively, and the origin is at the center of the intersection.
        ij is the i-th row and j-th column. xy(-wid_m/2, -hgt_m/2) corresponds to the upper left corner of row 0, column 0.
        '''
        x_arr = np.array(x_arr)
        y_arr = np.array(y_arr)
        i_arr = np.zeros(y_arr.shape)
        j_arr = np.zeros(x_arr.shape)
        j_arr = np.floor((x_arr + self.wid_m_half) / self.cell_size).astype(np.int16)
        i_arr = np.floor((y_arr + self.hgt_m_half) / self.cell_size).astype(np.int16)
        i_arr[i_arr<0] = 0
        j_arr[j_arr<0] = 0
        i_arr[i_arr>=self.i_n] = self.i_n-1
        j_arr[j_arr>=self.j_n] = self.j_n-1
        return [i_arr, j_arr]

    def init_ex_lane_record(self):
        ex_lane_record = {}
        for i in range(NS_lane_count):
            ex_lane_record['N' + str(i)] = [] # Each element is [veh_id, occ_start, occ_end]
            ex_lane_record['S' + str(i)] = []
        for i in range(EW_lane_count):
            ex_lane_record['E' + str(i)] = []
            ex_lane_record['W' + str(i)] = []
        return ex_lane_record
    
    def clear_veh_cell(self, veh_id):
        '''Clear all grids occupied by a certain vehicle veh_id'''
        self.cells[self.cells == veh_id] = -1

    def add_time_dimension(self):
        '''When the third dimension t is not enough, just double the size'''
        self.cells = np.concatenate((self.cells, -np.ones(self.cells.shape, dtype=np.int16)), axis=2)
    
    def dispose_passed_time(self, timestep):
        '''Clear all information from the past time (you can't go back in time anyway)'''
        if timestep - self.t_start > 20 and self.cells.shape[2] > (timestep - self.t_start) + 1: # If there is no latter condition, sometimes the third dimension of cells will be empty and there will be an infinite loop.
            self.cells = self.cells[:, :, (timestep - self.t_start):]
            self.t_start = timestep
        for key, value in self.ex_lane_record.items():
            for record in value:
                if record[2] < timestep:
                    value.remove(record) # Delete the time-lapsed information in the exit channel

class XuManager(BaseInterManager):
    def __init__(self):
        super().__init__()
        self.veh_info = [] # The element is (veh, report message)
        
    def receive_V2I(self, sender, message):
        if message['type'] == 'appear':
            self.update_topology()
        elif message['type'] == 'report':
            self.veh_info.append((sender, message))

    @staticmethod
    def is_conflict(ap_arm_dir_1, ap_arm_dir_2):
        return ap_arm_dir_1 in conflict_movements[ap_arm_dir_2]

    def update_topology(self):
        # Collect vehicle location information
        self.veh_info.clear()
        ComSystem.I_broadcast({'type': 'request report'})
        # Sort by x from large to small
        self.veh_info.sort(key=lambda e: -e[1]['inst_x'])
        # Insert virtual head car 0
        virtual_lead_x = self.veh_info[0][1]['inst_x'] + desired_cf_distance
        self.veh_info.insert(0, (None, {
            'inst_x': virtual_lead_x
        }))
        # Create conflict graph
        num_node = len(self.veh_info)
        cf_graph = nx.DiGraph()
        cf_graph.add_node(0, ap_arm_dir = 'Xx')
        for (i, info) in enumerate(self.veh_info):
            if i == 0:
                continue
            cf_graph.add_node(i, ap_arm_dir = info[1]['ap_arm'] + info[1]['turn_dir'])
            for j in range(1, i): # car in front j
                if XuManager.is_conflict(
                    cf_graph.nodes[j]['ap_arm_dir'], 
                    cf_graph.nodes[i]['ap_arm_dir']
                ):
                    cf_graph.add_edge(j, i)
        for node in cf_graph.nodes:
            if node > 0 and cf_graph.in_degree(node) == 0:
                cf_graph.add_edge(0, node)
        # plt.subplot(131)
        # nx.draw_shell(cf_graph, with_labels=True, font_weight='bold')
        # spanning tree
        tree = nx.DiGraph()
        tree.add_nodes_from(cf_graph)
        tree.nodes[0]['depth'] = 0
        for node in tree.nodes:
            if node == 0: 
                continue
            max_depth = 0
            max_depth_pred = 0
            for pred in cf_graph.predecessors(node):
                if tree.nodes[pred]['depth'] > max_depth: 
                    max_depth = tree.nodes[pred]['depth']
                    max_depth_pred = pred
            tree.nodes[node]['depth'] = max_depth + 1
            tree.add_edge(max_depth_pred, node)
        # plt.subplot(132)
        # nx.draw_shell(tree, with_labels=True, font_weight='bold')
        # Communication topology, this time it is an undirected graph, because the communication here is bidirectional
        com_graph = nx.Graph(tree)
        nodes_same_depth = [[] for i in range(num_node)]
        max_depth = 0
        for node in com_graph.nodes:
            depth = com_graph.nodes[node]['depth']
            max_depth = max(depth, max_depth)
            nodes_same_depth[depth].append(node)
        nodes_same_depth = nodes_same_depth[0:max_depth+1]
        for node_list in nodes_same_depth:
            for i in range(len(node_list)):
                for j in range(i + 1, len(node_list), 1):
                    com_graph.add_edge(node_list[i], node_list[j])
        # plt.subplot(133)
        # nx.draw_shell(com_graph, with_labels=True, font_weight='bold')
        # plt.show()
        # Generate several matrices A Q L
        A = nx.to_numpy_array(com_graph)
        A = np.asarray(A[1:, 1:]) # These matrices are all [1, N], excluding 0
        nghbor_0 = list(nx.neighbors(com_graph, 0))
        diag = np.zeros(num_node)
        diag[nghbor_0] = 1
        diag = diag[1: ]
        Q = np.diag(diag)
        L = -A
        for i in range(num_node-1):
            L[i, i] = np.sum(A[i, :]) - A[i, i]
        L_plus_Q = L + Q
        # print('L+Q=%s' % str(L_plus_Q))
        # Pass the coordination results to the car
        for (i, info) in enumerate(self.veh_info):
            if i == 0:
                continue
            neighbor_index = np.where(L_plus_Q[i-1, :] != 0)[0] # i is the node number, and the index in these matrices is smaller than 1 
            # print('%d, neightbor_index=%s' % (i, str(neighbor_index)))
            neighbor_list = [self.veh_info[int(n+1)][0] for n in neighbor_index]
            l_q_list = L_plus_Q[i-1, neighbor_index]
            # print('L+Q=%s' % str(l_q_list))
            message = {
                'type': 'coordination',
                'self_depth': tree.nodes[i]['depth'], 
                'virtual_lead_x': virtual_lead_x, 
                'virtual_lead_v': virtual_lead_v, 
                'neighbor_list': neighbor_list, 
                'l_q_list': l_q_list
            }
            ComSystem.I2V(info[0], message)

# Choose an implementation based on your settings
if inter_control_mode == 'traffic light':
    inter_manager = TrafficLightManager()
elif inter_control_mode == 'Dresner':
    inter_manager = DresnerManager()
elif inter_control_mode == 'Xu':
    inter_manager = XuManager()

# This isn't the best idea, but let's do this for now
import simulator

class ComSystem:
    @staticmethod
    def V2V(receiver, sender, message):
        receiver.receive_V2V(sender, message)

    @staticmethod   
    def V2I(sender, message):
        inter_manager.receive_V2I(sender, message)

    @staticmethod
    def I2V(receiver, message):
        receiver.receive_I2V(message)

    @staticmethod
    def I_broadcast(message):
        for group, vehs in simulator.Simulator.getInstance().all_veh.items():
            for veh in vehs:
                veh.receive_broadcast(message)
        
