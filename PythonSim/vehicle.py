import math
import logging

from lib.settings import arm_len, inter_v_lim, arm_v_lim, inter_control_mode, veh_dt, desired_cf_distance, kp, kv
from map import Track
from inter_manager import ComSystem
import random

class BaseVehicle:
    '''
    units: meter, second
    x is defined as the position of Front bumper
    '''
    def __init__(self, id, veh_param, cf_param, init_v, timestep):
        # static
        self._id = id
        self.veh_wid = veh_param['veh_wid']
        self.veh_len = veh_param['veh_len']
        self.veh_len_front = veh_param['veh_len_front']
        self.veh_len_back = self.veh_len - self.veh_len_front
        self.max_v = veh_param['max_v']
        self.max_acc = veh_param['max_acc']
        self.max_dec = veh_param['max_dec']
        self.track = Track(veh_param['ap_arm'], veh_param['ap_lane'], veh_param['turn_dir'])

        # dynamic
        self.timestep = timestep
        self.inst_a = 0
        self.inst_v = init_v
        self.zone = 'ap'                       # 'ap' = approach lane, 'ju' = junction area, 'ex' = exit lane
        self.inst_lane = veh_param['ap_lane']  # When zone == 'ap' or 'ex', the current lane
        self.inst_x = -arm_len                 # When zone == 'ap', it is (- the distance to the parking line); zone == 'ju', it is the distance traveled along the trajectory in the intersection; zone == 'ex', it is Distance traveled along exit road

        #Set car following parameters
        cf_param['v0'] = min(self.max_v, cf_param['v0'])
        self.cf_model = CFModel(cf_param=cf_param)

        # self.track.confirm_ex_lane(0)

    def __eq__(self, vehicle):  
        # Operator overloading, after testing, it seems that it can be passed to descendants
        return self._id == vehicle._id

    def acc_with_lead_veh(self, lead_veh):
        '''Only consider the acceleration when following a car'''
        if not lead_veh:
            # If there is no vehicle in front, assume that the vehicle in front is infinitely far away
            return self.cf_model.acc_from_model(self.inst_v, 1e3, self.cf_model.v0)
        else: 
            s = lead_veh.inst_x - self.inst_x - lead_veh.veh_len
            v_l = lead_veh.inst_v
            return self.cf_model.acc_from_model(self.inst_v, s, v_l)

    def update_control(self, lead_veh):
        '''Wait for it to be rewritten in a subclass, brother'''
        # Only consider the vehicle in front
        self.inst_a = self.acc_with_lead_veh(lead_veh)
        # According to vehicle performance, limit the maximum and minimum
        self.inst_a = min(max(self.inst_a, - self.max_dec), self.max_acc)

    def update_position(self, dt):
        '''It's equivalent to doing points and updating position and speed. Returns: whether the zone has changed'''
        self.timestep += 1

        if self.inst_v <= 0 and self.inst_a <= 0: # When the vehicle is stopped, the vehicle cannot reverse even if the acceleration is negative.
            self.inst_a = 0
        self.inst_x += self.inst_a * (dt ** 2) / 2 + self.inst_v * dt
        self.inst_v += self.inst_a * dt
        self.inst_v = min(max(self.inst_v, 0), self.max_v)

        if self.zone == 'ap' and self.inst_x >= 0:
            self.zone = 'ju'
            self.inst_lane = -1
            return True
        elif self.zone == 'ju' and self.inst_x >= self.track.ju_shape_end_x[-1]: 
            self.zone = 'ex'
            self.inst_x -= self.track.ju_shape_end_x[-1]
            self.inst_lane = self.track.ex_lane
            return True
        
        logging.debug("%d, %d, %s, %d, %.2f, %.2f, %.2f" % (self.timestep, self._id, self.zone, self.inst_lane, self.inst_x, self.inst_v, self.inst_a))
        return False

    def receive_broadcast(self, message):
        pass
    
    def receive_I2V(self, message):
        pass

class HumanDrivenVehicle(BaseVehicle):
    '''Manually driven cars only respond to traffic lights'''
    def __init__(self, id, veh_param, cf_param, init_v, timestep):
        super().__init__(id, veh_param, cf_param, init_v, timestep)
        
        # One lane situation
        self.track.confirm_ex_lane(0) # Temporarily single exit lane, which is 0

        # # 3 条车道的情况
        # if veh_param['turn_dir'] == 'l':
        #     self.track.confirm_ex_lane(0)
        # elif veh_param['turn_dir'] == 't':
        #     self.track.confirm_ex_lane(1)
        # else:
        #     self.track.confirm_ex_lane(2)

        self.traffic_light = None # control_order

        # Backup of car following parameters
        self.cf_v0_backup = None
        self.cf_T_backup = None

    def update_control(self, lead_veh):
        # Consider the car in front
        self.inst_a = super().acc_with_lead_veh(lead_veh)
        # When encountering a red or yellow light on the entrance road, you cannot cross the stop line
        if self.zone == 'ap' and self.traffic_light != 'G':
            self.inst_a = min(self.inst_a, self.cf_model.acc_from_model(self.inst_v, - self.inst_x - self.veh_len_front, 0))
        # Limit the maximum and minimum values
        self.inst_a = min(max(self.inst_a, - self.max_dec), self.max_acc)

    def update_position(self, dt):
        switch_group = super().update_position(dt)
        if self.zone == 'ap' and self.inst_x > -50 and not self.cf_v0_backup:
            # When the distance to the intersection is closer, the desired speed is changed to the intersection speed limit, and the headway is changed to a smaller value.
            self.cf_v0_backup = self.cf_model.v0
            self.cf_T_backup = self.cf_model.T
            self.cf_model.v0 = min(self.cf_model.v0, inter_v_lim)
            self.cf_model.T = min(self.cf_model.T, 1)
        elif self.zone == 'ex' and switch_group:
            # Just changed from the intersection area to the exit road, and changed back to the following parameters
            self.cf_model.v0 = self.cf_v0_backup
            self.cf_model.T = self.cf_T_backup
        return switch_group

    def receive_broadcast(self, message):
        if self.zone == 'ap':
            self.traffic_light = message[self.track.ap_arm + self.track.turn_dir]

    def receive_I2V(self, message):
        '''Don't listen to the bastard chanting sutra'''
        return

class DresnerVehicle(BaseVehicle):
    '''corresponds to the self-driving car in Dresner's article and responds to DresnerManager'''
    def __init__(self, id, veh_param, cf_param, init_v, timestep, faultyCar):
        super().__init__(id, veh_param, cf_param, init_v, timestep)
        #Control information
        self.reservation = None
        self.timeout = 0 # Unit: s
        # optimistic and pessimistic in the text
        self.optimism = True
        self.ap_acc_profile = None
        self.faultyCar = faultyCar
        self.crashOccured=False
        self.faultTime=0

    def plan_arr(self):
        '''According to the maximum arr_v, the earliest planned arrival time and speed of arr_t. Because only the leading car in a lane can plan this, so as long as the reservation is obtained, the plan can definitely be executed'''
        if self.inst_v < inter_v_lim:
            acc_distance = (inter_v_lim**2 - self.inst_v**2) / 2 / self.max_acc # The distance required to accelerate to v_lim
            if acc_distance >= (-self.inst_x):
                # If the acceleration is less than v_lim when reaching the parking line, accelerate the whole way.
                arr_v = math.sqrt(self.inst_v**2 + 2*self.max_acc*(-self.inst_x))
                t1 = (arr_v - self.inst_v) / self.max_acc
                arr_t = self.timestep + t1 / veh_dt
                self.ap_acc_profile = [[self.timestep, self.max_acc]]
            else:
                # It's enough to accelerate before reaching the parking line. Let's accelerate-constant speed for now. Acceleration-deceleration is too difficult to calculate.
                arr_v = inter_v_lim
                t1 = (inter_v_lim - self.inst_v) / self.max_acc
                t2 = ((-self.inst_x) - acc_distance) / inter_v_lim
                arr_t = self.timestep + (t1+t2) / veh_dt
                self.ap_acc_profile = [
                    [self.timestep, self.max_acc],
                    [self.timestep + t1/veh_dt, 0]
                ]
        else:
            # Now the speed exceeds v_lim, constant speed - slow down (there should not be a situation where the speed cannot be reduced)
            dec_distance = (inter_v_lim**2 - self.inst_v**2) / 2 / -(self.max_dec)
            if dec_distance > (-self.inst_x):
                print('Error: veh %d is unable to brake at stop bar' % self._id)
                arr_v = self.inst_v
                arr_t = self.timestep + ((-self.inst_x) / self.inst_v) / veh_dt
                self.ap_acc_profile = [[self.timestep, 0]]
            else:
                arr_v = inter_v_lim
                t2 = (self.inst_v - inter_v_lim) / self.max_dec
                t1 = ((-self.inst_x) - dec_distance) / self.inst_v
                arr_t = self.timestep + (t1+t2) / veh_dt
                self.ap_acc_profile = [
                    [self.timestep, 0],
                    [self.timestep + t1/veh_dt, -self.max_dec]
                ]
        return [arr_t, arr_v]
    
    def update_control(self, lead_veh):
        if self.zone == 'ap':
            if not lead_veh and not self.reservation and not self.crashOccured:
                # There is no car ahead, so make a reservation
                [arr_t, arr_v] = self.plan_arr()
                # logging.debug("veh %d, arr_t = %d, arr_v = %d, ap_acc_profile = %s" % (self._id, arr_t, arr_v, self.ap_acc_profile))
                ComSystem.V2I(self, {
                    'type': 'request',
                    'veh_id': self._id, 
                    'arr_t': arr_t, 
                    'arr_v': arr_v, 
                    'arr_arm': self.track.ap_arm,
                    'arr_lane': self.track.ap_lane, 
                    'turn_dir': self.track.turn_dir, 
                    'veh_len': self.veh_len, 
                    'veh_wid': self.veh_wid, 
                    'veh_len_front': self.veh_len_front,
                    'max_acc': self.max_acc,
                    'max_dec': self.max_dec
                })
            if self.reservation:
                # If the reservation is successful, the acceleration will be executed according to the previously calculated plan.
                for t, a in self.ap_acc_profile:
                    if self.timestep >= t:
                        self.inst_a = a
                # logging.debug("veh %d, according to ap_acc_profile, inst_a = %f" % (self._id, self.inst_a))
            else:
                # Unsuccessful，prepare to stop at stop bar & follow leading vehicle
                self.inst_a = min(self.acc_with_lead_veh(lead_veh), self.cf_model.acc_from_model(self.inst_v, - self.inst_x - self.veh_len_front, 0))
        elif self.zone == 'ju':
            # Check if the vehicle is marked as faulty
            if self.faultyCar and self.timestep >= self.faultTime:
                ComSystem.V2I(self, {
                    'type': 'fault',
                    'veh_id': self._id
                })
                # If the vehicle is faulty, initiate an immediate stop by applying maximum safe deceleration
                self.inst_a = -10
                # Optionally, log this event or take additional actions as necessary
                logging.info(f"Faulty vehicle {self._id} stopping in the intersection.")
            else:
                # If the vehicle is not faulty, run according to the acceleration requirements of the reservation
                if not self.crashOccured:
                    for t, a in self.reservation['acc']:
                        if self.timestep >= t:
                            self.inst_a = a
        else:
            # exit lane, perform normal car following
            super().update_control(lead_veh)
        # # For testing crashes
        # if self.inst_x > -50 and self.inst_v > 0:
        #     self.inst_a = -6

    def update_position(self, dt):
        switch_group = super().update_position(dt)
        if switch_group and self.zone == 'ex':
            ComSystem.V2I(self, {
                'type': 'done',
                'veh_id': self._id, 
                'res_id': self.reservation['res_id']
            })
        return switch_group

    def receive_I2V(self, message):
        if message['type'] == 'acknowledge':
            if message['res_id'] != self.reservation['res_id']:
                print('Error: ack message with [\'res_id\'] = %d is sent to veh %d' % (message['res_id'], self._id))
        elif message['type'] == 'confirm':
            self.reservation = message['reservation']
            self.track.confirm_ex_lane(self.reservation['ex_lane'])
            self.faultTime = random.uniform(float(self.reservation["arr_t"]), float(self.reservation["exit_time"]))
            if self.faultyCar:
                print("Start time is arr_t: ",self.reservation["arr_t"],"End time is exit_time: ",self.reservation["exit_time"])
                print(f"Faulty vehicle {self._id} will crash at time {self.faultTime}")
        elif message['type'] == 'reject':
            self.timeout = message['timeout']

    def receive_broadcast(self, message):
        if message["type"] =="crash":
            self.crashOccured=True
            if self.zone == 'ju':  # Check if the vehicle is in the junction zone
                # Set the vehicle's acceleration to the maximum safe deceleration rate
                self.inst_a = -self.max_dec
                # Optionally, log this event or take additional actions as necessary
                logging.info(f"Vehicle {self._id} stopping at max deceleration rate due to allStop broadcast.")
            elif self.zone == 'ap':  # Vehicle is in the approach zone
                # self.reservation = None  # Clear any existing reservations
                # Set acceleration to max deceleration rate to stop at the stop line
                # self.inst_a = -self.max_dec
                logging.info(f"Vehicle {self._id} in approach zone rejecting reservations and stopping due to allStop broadcast.")












class XuVehicle(BaseVehicle):
    def __init__(self, id, veh_param, cf_param, init_v, timestep):
        super().__init__(id, veh_param, cf_param, init_v, timestep)
        self.reported = False
        self.depth = None
        self.virtual_lead_x = None
        self.virtual_lead_v = None
        self.neighbor_list = None
        self.l_q_list = None

        # One lane situation
        self.track.confirm_ex_lane(0) # Temporarily single exit lane, which is 0

        # # 3 lane situation
        # if veh_param['turn_dir'] == 'l':
        #     self.track.confirm_ex_lane(0)
        # elif veh_param['turn_dir'] == 't':
        #     self.track.confirm_ex_lane(1)
        # else:
        #     self.track.confirm_ex_lane(2)

    def update_control(self, lead_veh):
        if self.depth and self.zone == 'ap':
            if lead_veh:
                a_1 = self.acc_with_lead_veh(lead_veh)
            else:
                a_1 = self.max_acc
            a_2 = self.acc_from_feedback()
            self.inst_a = min(a_1, a_2)
            self.inst_a = min(max(self.inst_a, - self.max_dec), self.max_acc)
            # logging.debug("Veh %d, a_1 = %.2f, a_2 = %.2f, inst_a = %.2f" % (self._id, a_1, a_2, self.inst_a))
            # self.inst_a = self.acc_from_feedback()
            # self.inst_a = min(max(self.inst_a, - self.max_dec), self.max_acc)
            # logging.debug("Veh %d, inst_a = %.2f" % (self._id, self.inst_a))
        else:
            super().update_control(lead_veh)
    
    def acc_from_feedback(self): 
        '''The final simplified result in Ch 3.4'''
        acc = 0
        for j in range(len(self.l_q_list)):
            x_bar_j_1 = (self.neighbor_list[j].inst_x - self.virtual_lead_x) - desired_cf_distance * (0 - self.neighbor_list[j].depth)
            x_bar_j_2 = self.neighbor_list[j].inst_v - self.virtual_lead_v
            acc += (-kp) * (self.l_q_list[j] * x_bar_j_1) + (-kv) * (self.l_q_list[j] * x_bar_j_2)
        return acc

    def update_position(self, dt):
        if not self.reported:
            if self.track.turn_dir != 'r':
                ComSystem.V2I(self, {'type': 'appear'})
            self.reported = True
            
        if self.virtual_lead_x and self.zone == 'ap':
            self.virtual_lead_x += self.virtual_lead_v * dt
        return super().update_position(dt)

    def receive_I2V(self, message):
        if message['type'] == 'coordination':
            self.depth = message['self_depth']
            self.virtual_lead_x = message['virtual_lead_x']
            self.virtual_lead_v = message['virtual_lead_v']
            self.neighbor_list = message['neighbor_list']
            self.l_q_list = message['l_q_list']

    def receive_broadcast(self, message):
        if message['type'] == 'request report':
            # Single lane will only be reported in approach zone
            if self.inst_x < 0:
            # # # Three lanes, right turn not reported
            # if self.inst_x < 0 and self.track.turn_dir in 'lt':
                ComSystem.V2I(self, {
                    'type': 'report', 
                    'veh_id': self._id,
                    'inst_x': self.inst_x, 
                    'ap_arm': self.track.ap_arm,
                    'ap_lane': self.track.ap_lane, 
                    'turn_dir': self.track.turn_dir,
                    'ex_lane': self.track.ex_lane
                })

# Choose an implementation based on your settings
if inter_control_mode == 'traffic light':
    Vehicle = HumanDrivenVehicle
elif inter_control_mode == 'Dresner':
    Vehicle = DresnerVehicle
elif inter_control_mode == 'Xu':
    Vehicle = XuVehicle

class CFModel:
    '''Car-following model, here we use IDM model'''
    def __init__(self, cf_param):
        self.v0 = cf_param['v0']    # desired speed
        self.T = cf_param['T']      # desired time headway
        self.s0 = cf_param['s0']    # minimum gap when stopped
        self.a = cf_param['a']      # acceleration rate
        self.b = cf_param['b']      # deceleration rate
        self.delta = 4

    def acc_from_model(self, v, s, v_l):
        '''
        v       speed
        s       gap with front vehicle
        v_l     speed of the leading vehicle
        '''
        s_star = self.s0 + max(0, v * self.T + v * (v - v_l) / 2 / math.sqrt(self.a * self.b))
        if v < self.v0:
            acc_free = self.a * (1 - (v / self.v0)**self.delta)
        else:
            acc_free = self.a * (1 - v / self.v0) # If v > v0, follow the 1st power
        dec = - self.a * (s_star / max(s, self.s0))**2 # Here we deal with the situation of s < s0, and calculate it according to s0
        acc = acc_free + dec
        return acc

