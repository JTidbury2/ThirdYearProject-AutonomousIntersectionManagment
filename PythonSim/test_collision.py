import pytest
import copy
import math

import simulator 
from vehicle import Vehicle
from lib.settings import veh_param,  cf_param, gen_init_v

#Note, must remove 
# from inter_manager import ComSystem
# from vehicle 
# And make crash happend = True in inter_manager
sim= simulator.Simulator.getInstance()
inter_manager = simulator.inter_manager
inter_manager.crash_happpened=True

def test_collision_fail():
        new_veh_param = copy.deepcopy(veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 1
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =0
        new_veh2.rl_y =0
        new_veh2.heading=0
        new_veh2.speed = 1
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == [0,1]

def test_collision_pass():
        new_veh_param = copy.deepcopy(veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 1
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =4
        new_veh2.rl_y =4
        new_veh2.heading=0
        new_veh2.speed = 1
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_pass2():
        new_veh_param = copy.deepcopy(veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 1
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =0
        new_veh2.rl_y =3
        new_veh2.heading=0
        new_veh2.speed = 1
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_pass3():
        new_veh_param = copy.deepcopy(veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 1
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =6
        new_veh2.rl_y =0
        new_veh2.heading=0
        new_veh2.speed = -1
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_pass4():
        new_veh_param = copy.deepcopy(veh_param)
        print("new_veh_param",new_veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 1
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =-6
        new_veh2.rl_y = 0
        new_veh2.heading=0
        new_veh2.speed = 1
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_fail2():
        new_veh_param = copy.deepcopy(veh_param)
        print("new_veh_param",new_veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 10
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =0
        new_veh2.rl_y =-1
        new_veh2.heading=0
        new_veh2.speed = 10
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == [0,1]

def test_collision_pass5():
        new_veh_param = copy.deepcopy(veh_param)
        print("new_veh_param",new_veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=math.pi/2
        new_veh1.speed = 10
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =3
        new_veh2.rl_y =0
        new_veh2.heading=math.pi/2
        new_veh2.speed = 10
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_fail3():
        new_veh_param = copy.deepcopy(veh_param)
        print("new_veh_param",new_veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 10
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =3
        new_veh2.rl_y =0
        new_veh2.heading=math.pi/2
        new_veh2.speed = 10
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == [0,1]

def test_collision_fail4():
        new_veh_param = copy.deepcopy(veh_param)
        print("new_veh_param",new_veh_param)
        new_veh_param["ap_arm"]="N"
        new_veh_param["ap_lane"]="N"
        new_veh_param["turn_dir"]="l"
        new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh1.rl_x =0
        new_veh1.rl_y =0
        new_veh1.heading=0
        new_veh1.speed = 10
        new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0,False,False)
        new_veh2.rl_x =-4
        new_veh2.rl_y =0
        new_veh2.heading=math.pi/2
        new_veh2.speed = 10
        tempList = [new_veh1, new_veh2]
        assert inter_manager.rl_check_for_collision(tempList) == []


def test_head_on_collision():
    new_veh_param = copy.deepcopy(veh_param)
    new_veh_param["ap_arm"] = "N"
    new_veh_param["ap_lane"] = "N"
    new_veh_param["turn_dir"] = "l"
    new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh1.rl_x = 0
    new_veh1.rl_y = 0
    new_veh1.heading = 0  # Facing East
    new_veh1.speed = 10
    new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh2.rl_x = 10
    new_veh2.rl_y = 0
    new_veh2.heading = math.pi  # Facing West
    new_veh2.speed = 10
    tempList = [new_veh1, new_veh2]
    assert inter_manager.rl_check_for_collision(tempList) == [0,1]

def test_collision_at_oblique_angles():
    new_veh_param = copy.deepcopy(veh_param)
    new_veh_param["ap_arm"] = "N"
    new_veh_param["ap_lane"] = "N"
    new_veh_param["turn_dir"] = "l"

    new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh1.rl_x = 0
    new_veh1.rl_y = 0
    new_veh1.heading = math.pi / 4  
    new_veh1.speed = 1
 
    new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh2.rl_x = 1 
    new_veh2.rl_y = 2  
    new_veh2.heading = 3 * math.pi / 4  
    new_veh2.speed = 1
    
    tempList = [new_veh1, new_veh2]
    assert inter_manager.rl_check_for_collision(tempList) == []

def test_collision_sharp_turn_vs_diagonal_approach():
    new_veh_param = copy.deepcopy(veh_param)
    new_veh_param["ap_arm"] = "N"
    new_veh_param["ap_lane"] = "N"
    new_veh_param["turn_dir"] = "l"
    
    new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh1.rl_x = 2
    new_veh1.rl_y = 0
    new_veh1.heading = math.pi / 2  
    new_veh1.speed = 1

    new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh2.rl_x = 0
    new_veh2.rl_y = 1
    new_veh2.heading = 3 * math.pi / 4 
    new_veh2.speed = 1

    tempList = [new_veh1, new_veh2]
    assert inter_manager.rl_check_for_collision(tempList) == [0, 1]

def test_parallel_close_proximity():
    new_veh_param = copy.deepcopy(veh_param)
    new_veh_param["ap_arm"] = "N"
    new_veh_param["ap_lane"] = "1"
    new_veh_param["turn_dir"] = "l"
    

    new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh1.rl_x = 10
    new_veh1.rl_y = 5
    new_veh1.heading = 0  
    new_veh1.speed = 2
    

    new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh2.rl_x = 10
    new_veh2.rl_y = 7 
    new_veh2.heading = 0
    new_veh2.speed = 2
    
    tempList = [new_veh1, new_veh2]
    assert inter_manager.rl_check_for_collision(tempList) == [0,1]

def test_crossing_paths_at_intersection():
    new_veh_param = copy.deepcopy(veh_param)
    new_veh_param["ap_arm"] = "N"
    new_veh_param["ap_lane"] = "1"
    new_veh_param["turn_dir"] = "l"
    
    new_veh1 = Vehicle(0, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh1.rl_x = -10
    new_veh1.rl_y = -15
    new_veh1.heading = math.pi / 2 
    new_veh1.speed = 1

    new_veh2 = Vehicle(1, new_veh_param, cf_param, gen_init_v, 0, False, False)
    new_veh2.rl_x = -8
    new_veh2.rl_y = 5
    new_veh2.heading = -math.pi / 2 
    new_veh2.speed = 1

    tempList = [new_veh1, new_veh2]
    assert inter_manager.rl_check_for_collision(tempList) == []

