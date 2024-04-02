######################### Global step size setting#################### ######
veh_dt = 0.1
disp_dt = 0.04
simu_t = 600 # simulation time = 10 min
time_wrap = 4
# disp_dt = veh_dt / time_wrap

########################## Simulation Crash Params##################### #####
crashValues={"crashOccured": False}
########################## Scene parameter settings##################### #####
lane_width = 4
turn_radius = 5 # The American Urban Street Design Guidelines require that the corner radius of general urban road intersections should be 3~4.5m
arm_len = 100
NS_lane_count = 3
EW_lane_count = 3
arm_v_lim = 16.66 # 60 km/h
inter_v_lim = 11.11 # Speed ​​limit in intersection area, Human/Dresner/Xu will all use it
########################## Vehicle parameter settings##################### #####
veh_param = {
    'veh_wid': 2,
    'veh_len': 5,
    'veh_len_front': 1, # The distance from the front bumper to the front axle
    'max_v': 33,
    'max_acc': 3,
    'max_dec': 4,
    'ap_arm': None,
    'ap_lane': None,
    'turn_dir': None
}

cf_param = {
    'v0': 15, # 54 km/h, which is in line with the speed of ordinary urban main roads
    'T': 1.4,
    's0': 1.5,
    'a': 1.5,
    'b': 3
}

######################### Boundary condition settings##################### #####

# Generate the rules for vehicles, the traffic flow of vehicles on each entrance road and each turn. Unit: pcu/hour
veh_gen_rule_table = { 
    # 3 lane situation
    'Nl': [0    , 0    , 0    ],
    'Nt': [0    , 0    , 0    ],
    'Nr': [0    , 0    , 0    ],
    'Sl': [0    , 0    , 0    ],
    'St': [0    , 0    , 0    ],
    'Sr': [0    , 0    , 0    ],
    'El': [1800 , 0    , 0    ], 
    'Et': [0    , 1800 , 0    ], 
    'Er': [0    , 0    , 1800 ], 
    'Wl': [0    , 0    , 0    ],
    'Wt': [0    , 0    , 0    ], 
    'Wr': [0    , 0    , 0    ],
}
# The initial speed when the vehicle is generated (the speed of driving on the road segment)
gen_init_v = cf_param['v0']
# The shortest distance and time distance between two consecutive vehicles on a lane, unit: meters, seconds
min_gen_hs = veh_param['veh_len'] + cf_param['s0'] + gen_init_v**2 / 2 / veh_param['max_dec']
min_gen_ht = min_gen_hs / gen_init_v
        
######################### Intersection control scheme settings################### #######
inter_control_mode = 'Dresner' # 'traffic light', 'Dresner', 'Xu'

########################## Simulation parameters of the signal light#################### ######
phase = [
            [420, 'Nt', 'St'], 
            [420, 'Et', 'Wt']

            # [150, 'Nl', 'Nt', 'Nr'], 
            # [280, 'Sl', 'St', 'Sr'], 
            # [280, 'El', 'Et', 'Er'], 
            # [540, 'Wl', 'Wt', 'Wr']

            # [100, 'Nl', 'Nt', 'Nr'], 
            # [190, 'Sl', 'St', 'Sr'], 
            # [190, 'El', 'Et', 'Er'], 
            # [380, 'Wl', 'Wt', 'Wr']

            # [150, 'Nl', 'Sl', 'Nr', 'Sr', 'Er', 'Wr'],
            # [220, 'Nt', 'St', 'Nr', 'Sr', 'Er', 'Wr'],
            # [150, 'El', 'Wl', 'Nr', 'Sr', 'Er', 'Wr'],
            # [220, 'Et', 'Wt', 'Nr', 'Sr', 'Er', 'Wr']
        ]
yellow_time = 1.5

########################## Dresner’s simulation parameters#################### ######
inter_v_lim_min = 4 # The slowest speed through the intersection in the Dresner scheme

########################## Xu’s simulation parameters#################### ######
desired_cf_distance = 25 # Take 25 for single lane and 39 for three lanes
virtual_lead_v = inter_v_lim
kp = 0.15
kv = 0.7

# conflict_movements = {
# 'Nt': ['Nt', 'Wt', 'Et'],
# 'St': ['St', 'Wt', 'Et'],
# 'Wt': ['Nt', 'St', 'Wt'],
# 'Et': ['Nt', 'St', 'Et']
# }
# conflict_movements = {
# # For single-lane situations, the same entrance road and exit road both have diverging and merging conflicts.
# # ['Nl', 'Nt', 'Nr', 'Sl', 'St', 'Sr', 'Wl', 'Wt', 'Wr', 'El', 'Et', 'Er' ]
# 'Nl': ['Nl', 'Nt', 'Nr', 'St', 'Sr', 'Wl', 'Wt', 'El', 'Et'],
# 'Nt': ['Nl', 'Nt', 'Nr', 'Sl', 'Wl', 'Wt', 'Wr', 'El', 'Et'],
# 'Nr': ['Nl', 'Nt', 'Nr', 'Sl', 'Et'],
# 'Sl': ['Nt', 'Nr', 'Sl', 'St', 'Sr', 'Wl', 'Wt', 'El', 'Et'],
# 'St': ['Nl', 'Sl', 'St', 'Sr', 'Wl', 'Wt', 'El', 'Et', 'Er'],
# 'Sr': ['Nl', 'Sl', 'St', 'Sr', 'Wt'],
# 'Wl': ['Nl', 'Nt', 'Sl', 'St', 'Wl', 'Wt', 'Wr', 'Et', 'Er'],
# 'Wt': ['Nl', 'Nt', 'Sl', 'St', 'Sr', 'Wl', 'Wt', 'Wr', 'El'],
# 'Wr': ['Nt', 'Wl', 'Wt', 'Wr', 'El'],
# 'El': ['Nl', 'Nt', 'Sl', 'St', 'Wt', 'Wr', 'El', 'Et', 'Er'],
# 'Et': ['Nl', 'Nt', 'Nr', 'Sl', 'St', 'Wl', 'El', 'Et', 'Er'],
# 'Er': ['St', 'Wl', 'El', 'Et', 'Er']
# }
conflict_movements = {
    # For three lanes, there are no conflicts between diverging and merging.
    # ['Nl', 'Nt', 'Nr', 'Wl', 'Wt', 'Wr', 'Sl', 'St', 'Sr', 'El', 'Et', 'Er']
    'Nl': ['Nl', 'Wl', 'St', 'El', 'Et'],
    'Nt': ['Nt', 'Wl', 'Wt', 'Sl', 'Et'], 
    'Nr': ['Nr'], 
    'Sl': ['Nt', 'Wl', 'Wt', 'Sl', 'El'], 
    'St': ['Nl', 'Wt', 'St', 'El', 'Et'], 
    'Sr': ['Sr'], 
    'Wl': ['Nl', 'Nt', 'Wl', 'Sl', 'Et'], 
    'Wt': ['Nt', 'Wt', 'Sl', 'St', 'El'], 
    'Wr': ['Wr'], 
    'El': ['Nl', 'Wt', 'Sl', 'St', 'El'], 
    'Et': ['Nl', 'Nt', 'Wl', 'St', 'Et'], 
    'Er': ['Er']
}