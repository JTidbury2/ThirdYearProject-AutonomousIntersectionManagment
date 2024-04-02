import sys
import time
import logging
import os

import lib.settings
from rl_agent import AgentInference


env_config = '../rl-agents/scripts/configs/IntersectionEnv/env_3way_int.json'
agent_config = '../rl-agents/scripts/configs/IntersectionEnv/agents/DQNAgent/ego_attention_8h.json'
model_path = '../PettingZooSim/HighwayEnv/out/ThreeWayIntersectionEnv/DQNAgent/run_20240312-173947_14944/checkpoint-final.tar'
rl_agent_export= AgentInference(env_config, agent_config, model_path)



def exec_simulation():
    from PyQt5.QtWidgets import QApplication
    from my_main_window import MyMainWindow
    from cal_delay import cal_metrics

    # Define the log directory and file name
    print('Current working directory:')
    print(os.getcwd())
    log_dir = 'log'
    log_fname = os.path.join(log_dir, 'log %s.log' % time.strftime("%Y-%m-%d %H-%M-%S"))


    # Check if the log directory exists, and create it if it doesn't
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)


    logging.basicConfig(filename=log_fname, format='%(message)s', level=logging.DEBUG)
    logging.debug('t, veh._id, zone, lane, x, v, a')
    print(log_fname)




    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    app.exec_()

    # print('Simulation finished, banana')

    print('Calculating metrics...')
    print(os.path.exists(log_fname))

    metrics = cal_metrics(log_fname)
    for key, value in metrics.items():
        print(key, '=', value)
    print('')

if __name__ == '__main__':
    # Adjustment plan
    mode = sys.argv[1]
    if mode == 'Dresner':
        lib.settings.arm_len = 100
        lib.settings.inter_control_mode = 'Dresner'
    elif mode == 'Xu':
        lib.settings.arm_len = 200
        lib.settings.inter_control_mode = 'Xu'
    else:
        lib.settings.arm_len = 100
        lib.settings.inter_control_mode = 'traffic light'

    #Adjust traffic
    total_flow = int(sys.argv[2])

    random = True if sys.argv[3] == 1 else False
    lib.settings.liveValues["random"]=random


    #    # One lane is straight only
    #t_flow = total_flow / 4
    # Balance
    if not random:
        l_flow = total_flow / 16
        t_flow = total_flow / 8
        r_flow = total_flow / 16
    else:
        # Randomize traffic flow proportions
        l_flow = total_flow * random.random() / 8
        t_flow = total_flow * random.random() / 4
        r_flow = total_flow * random.random() / 8
    # # unbalanced
    # N_flow = total_flow / 9
    # S_flow = total_flow / 9 * 2
    # E_flow = total_flow / 9 * 2
    # W_flow = total_flow / 9 * 4

    lib.settings.veh_gen_rule_table = {
        # Three lane balance
        'Nl': [l_flow, 0, 0], 
        'Nt': [0, t_flow, 0], 
        'Nr': [0, 0, r_flow], 
        'Sl': [l_flow, 0, 0], 
        'St': [0, t_flow, 0], 
        'Sr': [0, 0, r_flow], 
        'El': [l_flow, 0, 0], 
        'Et': [0, t_flow, 0], 
        'Er': [0, 0, r_flow], 
        'Wl': [l_flow, 0, 0], 
        'Wt': [0, t_flow, 0], 
        'Wr': [0, 0, r_flow]
    }
    print('## %d = 4 * (%d + %d + %d)' % (total_flow, l_flow, t_flow, r_flow))
    print(lib.settings.veh_gen_rule_table)


    exec_simulation()

