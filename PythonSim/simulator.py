import copy
import logging

from vehicle import Vehicle
from inter_manager import inter_manager
from lib.settings import arm_len, veh_dt, veh_param, cf_param, NS_lane_count, EW_lane_count, veh_gen_rule_table, min_gen_hs, gen_init_v, crashValues
import random

import numpy as np

from rl_agent import AgentInference, VehicleInterface

class Simulator:
    _instance = None

    @staticmethod 
    def getInstance():
        if Simulator._instance == None:
            Simulator()
        return Simulator._instance
    
    def get_sim_over(self):
        return self.sim_over

    def __init__(self):
        if Simulator._instance != None:
            raise Exception("This class is a singleton, but more than one objects are created.")
        else:
            Simulator._instance = self

        self.timestep = 0
        self.random_count = random.randint(25, 35)
        self.crash_count=0
        self.crash_time=2000
        self.sim_over=False
        self.rl_swap = False
        self.rl_OBS_COUNT = 15
        self.veh_rl_values = {}
        self.veh_rl_obs = {}
        self.veh_rl_actions = {}
        self.veh_rl_updated_values = {}
        self.rl_agent = None
        self.rl_vehicle = None 

        self.set_up_rlAgent()

        # Generate vehicle counts for vehicle ID assignment
        self.gen_veh_count = 0  
        # For vehicle queues that are not placed in the simulation area for each lane, refer to the Meng2018Analysis article
        self.point_queue_table = self.init_point_queue_table()
        # Vehicles in the simulation area
        self.all_veh = {
            'Nap': [],
            'Sap': [],
            'Eap': [],
            'Wap': [],
            'Nex': [],
            'Sex': [],
            'Eex': [],
            'Wex': [],
            'ju': []
        }
        self.vehicleCount=0

        

    def set_up_rlAgent(self):
        env_config = '../rl-agents/scripts/configs/IntersectionEnv/env_3way_int.json'
        agent_config = '../rl-agents/scripts/configs/IntersectionEnv/agents/DQNAgent/ego_attention_8h.json'
        model_path = '../PettingZooSim/HighwayEnv/out/ThreeWayIntersectionEnv/DQNAgent/run_20240312-173947_14944/checkpoint-final.tar'
        self.rl_agent= AgentInference(env_config, agent_config, model_path)

    def set_up_rlVehicle(self):
        temp = np.array([0,0])
        self.rl_vehicle = VehicleInterface(temp,0,0)


    def rl_update(self):
        for veh in self.all_veh["ju"]:
            if veh._id in self.veh_rl_values:
                self.veh_rl_values[veh._id] = veh.get_veh_rl_values()
        if self.rl_swap:
            self.rl_get_obs()
            self.rl_get_action()
            self.rl_update_pos_after_action()
            

    def rl_update_pos_after_action(self):
        for veh in self.all_veh["ju"]:
            temp= np.array([self.veh_rl_values[veh._id]["x"],self.veh_rl_values[veh._id]["y"]])
            self.rl_vehicle.update_vehicle_values(temp, self.veh_rl_values[veh._id]["v"], self.veh_rl_values[veh._id]["h"])
            self.veh_rl_updated_values[veh._id] = self.rl_vehicle.get_state(self.veh_rl_actions[veh._id])
            self.veh_rl_updated_values[veh._id]["veh_id"] = veh._id



    def rl_get_obs(self):
        for ego_veh in self.all_veh["ju"]:
            obs = []
            sorted_veh = sorted(self.all_veh["ju"], key=lambda veh: self.get_distance(veh.rl_x, veh.rl_y, ego_veh.rl_x, ego_veh.rl_y))
            count=0
            for veh in sorted_veh:
                if count < self.rl_OBS_COUNT:
                    veh_rl_values = self.veh_rl_values[veh._id]
                    obs.append([veh_rl_values["presence"], veh_rl_values["x"], veh_rl_values["y"], veh_rl_values["vx"], veh_rl_values["vy"], veh_rl_values["cos_h"], veh_rl_values["sin_h"]])
                    count+=1
                else:
                    break

            while count < self.rl_OBS_COUNT:
                obs.append([0] * len(obs[0]))
                count+=1

            self.veh_rl_obs[ego_veh._id] = obs



    def get_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    

    def rl_get_action(self):
        for veh in self.all_veh["ju"]:
            actions= self.rl_agent.get_agent_action(self.veh_rl_obs[veh._id])
            self.veh_rl_actions[veh._id] = actions








    def update(self):
        self.check_for_collisions()
        self.timestep += 1
        # print('Timestep: %d' % self.timestep)
        to_switch_group = self.all_update_position()
        # print('all_update_position')
        self.update_group(to_switch_group)
        # print('update_group(to_switch_group)')
        self.gen_new_veh()
        # print('gen_new_veh')
        self.remove_out_veh()
        # print('remove_out_veh')
        self.update_all_control()
        # print('update_all_control')

        inter_manager.update()

        if self.check_for_finish():
            #finsh the simulation
            #insert code here 
            self.sim_over=True



    def switch_to_rl(self):
        for group, vehs in self.all_veh.items():
            if group == "ju":
                for veh in vehs:
                    veh.switch_to_rl()



    def check_for_collisions(self):
        crashed_vehicles= inter_manager.check_for_collision(self.all_veh["ju"])
        
        self.crash_count= len(crashed_vehicles)
        if inter_manager.check_for_collision_noCars() and self.crash_time==2000:
            self.crash_time=self.timestep+200


        if self.crash_count>0:
            # print(crashed_vehicles)
            pass

    def check_for_finish(self):
        if self.timestep >= self.crash_time:
            return True
        

    def all_update_position(self):
        '''For the vehicles in all_veh, update the location and record the vehicles whose grouping has changed'''
        to_switch_group = []
        for group, vehs in self.all_veh.items():
            for veh in vehs:
                switch_group = veh.update_position(veh_dt)
                if switch_group:
                    to_switch_group.append([veh, group])
        return to_switch_group
    
    def update_group(self, to_switch_group):
        '''Update the group of vehicles and sort them by location within the lane'''
        for veh, old_group in to_switch_group:
            if veh.zone == 'ju':
                new_group = 'ju'
            elif veh.zone == 'ex':
                new_group = str(veh.track.ex_arm) + 'ex'
            self.all_veh[new_group].append(veh)
            self.all_veh[old_group].remove(veh)

        for group, vehs in self.all_veh.items():
            vehs.sort(key=lambda veh: veh.inst_x) # Sort by x from small to large

    def remove_out_veh(self):
        '''Delete vehicles that run out of the simulation area'''
        to_delete = []
        for group, vehs in self.all_veh.items():
            if group[-2:] == 'ex':
                for veh in vehs:
                    if veh.inst_x >= arm_len + veh.veh_len_back:
                        to_delete.append([group, veh])
        for group, veh in to_delete:
            self.all_veh[group].remove(veh)

    def init_point_queue_table(self):
        point_queue_table = {}
        for i in range(NS_lane_count):
            point_queue_table['N' + str(i)] = [] # #The elements in the list are the steering directions of each vehicle to be generated
            point_queue_table['S' + str(i)] = []
        for i in range(EW_lane_count):
            point_queue_table['E' + str(i)] = []
            point_queue_table['W' + str(i)] = []
        return point_queue_table       

    def gen_new_veh(self): 
        '''Generate new vehicles in point_queue according to probability, and if feasible, put a vehicle into the simulation area'''
        for ap_arm in 'NSEW': # Each import road
            for turn_dir in 'lrt': # All directions
                flows = veh_gen_rule_table[ap_arm + turn_dir]
                for (lane, flow) in enumerate(flows):  # lane lane
                    prob = flow / 3600 * veh_dt
                    if np.random.rand() < prob:
                        self.point_queue_table.get(ap_arm + str(lane),[]).insert(0, turn_dir)
        for ap_arm_lane, queue in self.point_queue_table.items():
            ap_arm = ap_arm_lane[0]
            lane = int(ap_arm_lane[1])
            latest_veh = None 
            for some_veh in self.all_veh[ap_arm + 'ap']:
                if some_veh.inst_lane == lane:
                    latest_veh = some_veh
                    break
            if not latest_veh or (latest_veh.inst_x - (-arm_len)) > min_gen_hs:
                if len(queue) > 0: 
                    new_veh = self.make_veh(ap_arm, lane, queue.pop())
                    self.all_veh[ap_arm + 'ap'].insert(0, new_veh)
                    
    def make_veh(self, ap_arm, ap_lane, turn_dir):
        '''Create a vehicle object and return'''
        self.vehicleCount+=1
        new_veh_param = copy.deepcopy(veh_param)
        new_veh_param['ap_arm'] = ap_arm
        new_veh_param['ap_lane'] = ap_lane
        new_veh_param['turn_dir'] = turn_dir
        faultCar=False
 

        if self.vehicleCount == self.random_count:
            faultCar = True


        new_veh = Vehicle(self.gen_veh_count, new_veh_param, cf_param, gen_init_v, self.timestep,faultCar,crashValues["crashOccured"])
        self.gen_veh_count += 1
        return new_veh

    def update_all_control(self):
        '''All vehicles update their vehicle control status independently'''
        for group, vehs in self.all_veh.items():
            for (i, veh) in enumerate(vehs):
                lead_veh = None
                if group != 'ju': # entrance and exit
                    for j in range(i+1, len(vehs)):
                        if vehs[j].inst_lane == veh.inst_lane:
                            lead_veh = vehs[j]
                            break
                else: # At the intersection, there is no concept of lanes here, but in order to prevent two cars with the same trajectory from colliding, they are still considered to be in the same lane.
                    for j in range(i+1, len(vehs)):
                        if vehs[j].track.ap_arm == veh.track.ap_arm and vehs[j].track.ap_lane == veh.track.ap_lane \
                            and vehs[j].track.turn_dir == veh.track.turn_dir and vehs[j].track.ex_lane == veh.track.ex_lane:
                            lead_veh = vehs[j]
                            break
                veh.update_control(lead_veh)



    