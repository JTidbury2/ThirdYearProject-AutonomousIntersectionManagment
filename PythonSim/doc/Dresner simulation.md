# Dresner 2008 multiagent
+ area 250 m × 250 m, intersection located at the center 
+ intersection size determined by the number of lanes, which is variable

Each step: 
1. Probabilistically spawns new vehicles
2. *Provides sensor input to all vehicles*
3. Allows all driver agents to act
4. Updates the position of all vehicles according to the physical model
5. Removes any vehicles outside the simulated area that have completed their journey

Vehicles:
+ Vehicle Identification Number (VIN)
+ Length, Width
+ *Distance from front of vehicle to front axle*
+ *Distance from front of vehicle to rear axle*
+ max v, max a, min a, *max steer angle, sensor range*
+ Position
+ Velocity
+ *Heading*
+ Acceleration
+ *Steering angle*

The driver agent may access several simulated external sensors:
+ a list of all vehicles within the sensor range, I directly gave the vehicle in front of the vehicle, he didn’t care about the vehicle behind him anyway
+ a simplified laser range finder, I gave it directly to the car in front and knew its location

+ *lane keeping I can omit it here, I think, just control it horizontally*
+ *optimistic / pessimistic See algorithm3 and combine it with IDM*
    + An optimistic agent makes a reservation assuming it will arrive at the intersection in the minimum possible time
    + finds itself no longer stuck behind a slower vehicle will become optimistic and attempt to make a new, earlier reservation
    + pessimistic agent assumes it will be stuck at its current velocity until it reaches the intersection
    + has to cancel its reservation because there is no way for it to arrive on time, it becomes pessimistic

+ *arrival time estimation*

*model vehicle planar vehicle kinematics: dx/dt, dy/dt, d orientation/dt can be used to calculate the turning radius, I have omitted it here*

**delay**: 
+ When the vehicle is removed from simulation, its total delay is calculated 
+ compare the delays of all vehicles to delays using a policy that allows vehicles through the intersection unhindered

message:
+ Driver agents can send: Request, *Change-Request, (all properties) Cancel,* and Done (only VID)
+ Intersection send: 
    + Confirm: a unique identifier for the reservation, a start time, a start lane, a departure lane, restrictions on acc
    + Reject
    + Acknowledge: to Cancel and Done
    + *Emergency-Stop*

```python
request_message = {
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
}

change_request_message = {
    'type': 'change_request',
    'veh_id': veh._id, 
    'arr_t': 0, 
    'arr_v': 0, 
    'arr_arm': self.track.ap_arm,
    'arr_lane': ap_lane, 
    'turn_dir': veh.track.turn_dir, 
    'veh_len': veh.veh_len, 
    'veh_wid': veh.veh_wid
}

cancel_message = {
    'type': 'cancel', 
    'veh_id': 0, 
    'res_id': 0
}

done_message = {
    'type': 'done',
    'veh_id': 0, 
    'res_id': 0
}

confirm_message = {
    'type': 'confirm',
    'reservation': {
        'res_id': 0, 
        'ex_lane': 0,
        'arr_t': 0,
        'arr_v': 0,
        'acc': [
            [start_time1, acc1], 
            [start_time2, acc2]
        ]
    }
}

reject_message = {
    'type': 'reject',
    'timeout': 1
}

acknowledge_message = {
    'type': 'acknowledge',
    'res_id': 0
}
```

Outbound Lane: determine, priority. Turn left or right to find the nearest lane first, then go straight to find the corresponding lane.
Acceleration: Two profiles, first accelerate to the maximum and then maintain it, and then pass at a constant speed at that speed
*Reservation Distance: This estimate may not be reliable*
*timeout: pretty good*
**edge tile: The patching method is very rough, maybe I can just set a time interval on the exit path**

full simulator:
+ 3 lanes in each of the 4 cardinal directions
+ *turn with probability 0.1, left = right I'd better adjust the parameters here*
+ right lane -> turn right, etc
+ increasing granularity beyond twice the number of lanes can improve performance even more

'''
For a list of suitable exit lanes: (turn left: 012..., turn right: ...210, go straight: l, l+1, l-1...)
    Get the track from the map through get_ju_track
    Try two strategies: accelerate to maximum - constant speed / constant speed
        Each step:
            Calculate vehicle xy coordinates and direction
            Calculate the xy coordinates of the vehicle's dots in the logical coordinate system (rotate first, then place xy)
            Use grid.xy_to_ij to convert it into the cell occupied by the vehicle at this time
            Check if these cells are all -1
            if is:
                occupy these cells
            else:
                Use grid.clear_veh_cell to clear the reservation for this car (marked during the previous calculation)
                This planning failed
'''

    



            


