from typing import Dict, Text, Tuple

import numpy as np

from highway_env import utils
from highway_env.envs.common.abstract import AbstractEnv, MultiAgentWrapper
from highway_env.road.lane import AbstractLane, CircularLane, LineType, StraightLane
from highway_env.road.regulation import RegulatedRoad
from highway_env.road.road import RoadNetwork
from highway_env.vehicle.kinematics import Vehicle



class ThreeWayIntersectionEnv(AbstractEnv):
    #Simialr to intersection env but with 3 way intersection,
    #  the left lane is a circular lane to the left
    #  the right lane is a circular lane to the right
    #  the middle lane is a straight lane 


    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config.update(
            {
                "observation": {
                    "type": "MultiAgentObservation",
                    "observation_config":
                        {"type": "Kinematics",
                        "vehicles_count": 15,
                        "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
                        "features_range": {
                            "x": [-100, 100],
                            "y": [-100, 100],
                            "vx": [-20, 20],
                            "vy": [-20, 20],
                        },
                        "absolute": True,
                        "flatten": False,
                        "see_behind": True,
                        "observe_intentions": False,
                        "order": "sorted",
                        "normalize": False, }
                },
                "action": {
                    "type": "MultiAgentAction",
                    "action_config": {
                        "type": "DiscreteAction",
                        "actions_per_axis": 10,
                        "acceleration_range": [-9.5, 5.0],
                        "steering_range": [-0.7, 0.7]
                }},
                "actions_per_axis": 10,
                "duration": 26,  # [s]
                "controlled_vehicles": 3,
                "initial_vehicle_count":60,
                "spawn_probability": 1,
                "screen_width": 1200,
                "screen_height": 1200,
                "centering_position": [0.5, 0.6],
                "scaling": 5.5 * 1.3,
                "collision_reward": -15,
                "high_speed_reward": 0.5,
                "arrived_reward": 0,
                "reward_speed_range": [-0.5, 0.5],
                "normalize_reward": True,
                "offroad_terminal": True,
                "alive_reward":2
            }
        )
        return config
    
    def _reward(self, action: int) -> float:
        """Aggregated reward, for cooperative agents."""
        return sum(
            self._agent_reward(action, vehicle) for vehicle in self.controlled_vehicles
        ) / len(self.controlled_vehicles)

    def _rewards(self, action: int) -> Dict[Text, float]:
        """Multi-objective rewards, for cooperative agents."""
        agents_rewards = [
            self._agent_rewards(action, vehicle) for vehicle in self.controlled_vehicles
        ]
        return {
            name: sum(agent_rewards[name] for agent_rewards in agents_rewards)
            / len(agents_rewards)
            for name in agents_rewards[0].keys()
        }

    def _agent_reward(self, action: int, vehicle: Vehicle) -> float:
        """Per-agent reward signal."""
        rewards = self._agent_rewards(action, vehicle)
        reward = sum(
            self.config.get(name, 0) * reward for name, reward in rewards.items()
        )
        reward = self.config["arrived_reward"] if rewards["arrived_reward"] else reward
        reward *= rewards["on_road_reward"]
        if self.config["normalize_reward"]:
            reward = utils.lmap(
                reward,
                [self.config["collision_reward"], self.config["arrived_reward"]],
                [0, 1],
            )
        return reward

    def _agent_rewards(self, action: int, vehicle: Vehicle) -> Dict[Text, float]:
        """Per-agent per-objective reward signal."""
        is_in_speed_range = self.config["reward_speed_range"][0] <= vehicle.speed <= self.config["reward_speed_range"][1]
        return {
            "collision_reward": vehicle.crashed,
            "high_speed_reward": is_in_speed_range,
            "arrived_reward": self.has_arrived(vehicle),
            "on_road_reward": vehicle.on_road,
            "alive_reward":1,
        }
    
    def _is_terminated(self) -> bool:
        if (any(vehicle.crashed for vehicle in self.controlled_vehicles)):
            print("Terminated, crashed",self.time)
        elif (all(self.has_arrived(vehicle) for vehicle in self.controlled_vehicles)):
            print("Terminated, arrived",self.time)
        elif (self.config["offroad_terminal"] and not all(vehicle.on_road for vehicle in self.controlled_vehicles)):
            print("Terminated, offroad",self.time)

        return (
            any(vehicle.crashed for vehicle in self.controlled_vehicles)
            or (self.config["offroad_terminal"] and not all(vehicle.on_road for vehicle in self.controlled_vehicles))
        )

    def _agent_is_terminal(self, vehicle: Vehicle) -> bool:
        """The episode is over when a collision occurs or when the access ramp has been passed."""
        return vehicle.crashed or self.has_arrived(vehicle)

    def _is_truncated(self) -> bool:
        """The episode is truncated if the time limit is reached."""
        print("Time", self.time)
        print("Truncated", self.time >= self.config["duration"])
        return self.time >= self.config["duration"]

    def _info(self, obs: np.ndarray, action: int) -> dict:
        info = super()._info(obs, action)
        info["agents_rewards"] = tuple(
            self._agent_reward(action, vehicle) for vehicle in self.controlled_vehicles
        )
        info["agents_dones"] = tuple(
            self._agent_is_terminal(vehicle) for vehicle in self.controlled_vehicles
        )
        return info

    def _reset(self) -> None:
        self._make_road()
        self._make_vehicles(self.config["initial_vehicle_count"])

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
        obs, reward, terminated, truncated, info = super().step(action)
        self._clear_vehicles()
        self._spawn_vehicle(spawn_probability=self.config["spawn_probability"])
        return obs, reward, terminated, truncated, info
    
    def _make_road(self) -> None:
        """
        Make an 4-way intersection.

        The horizontal road has the right of way. More precisely, the levels of priority are:
            - 3 for horizontal straight lanes and right-turns
            - 1 for vertical straight lanes and right-turns
            - 2 for horizontal left-turns
            - 0 for vertical left-turns

        The code for nodes in the road network is:
        (o:outer | i:inner + [r:right, l:left]) + (0:south | 1:west | 2:north | 3:east)

        :return: the intersection road
        """
        lane_width = AbstractLane.DEFAULT_WIDTH
        right_turn_radius = lane_width + 5  # [m}
        left_turn_radius = right_turn_radius + 3*lane_width  # [m}
        outer_distance = right_turn_radius + lane_width / 2
        access_length = 50 + 50  # [m]

        net = RoadNetwork()
        n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
        for corner in range(4):
            angle = np.radians(90 * corner)
            is_horizontal = corner % 2
            priority = 3 if is_horizontal else 1
            rotation = np.array(
                [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
            )
            # Incoming
            # start = rotation @ np.array(
            #     [lane_width / 2, access_length + outer_distance]
            # )
            # end = rotation @ np.array([lane_width / 2, outer_distance])
            # net.add_lane(
            #     "o" + str(corner),
            #     "ir" + str(corner),
            #     StraightLane(
            #         start, end, line_types=[c, c], priority=priority, speed_limit=10
            #     ),
            # )
            # Incoming
            # Edited code os that there is 3 incomeing lanes instead of 1 
            for i in range(3):
                start = rotation @ np.array(
                    [lane_width / 2 + (i * lane_width), access_length + outer_distance + (2*lane_width)]
                )
                end = rotation @ np.array([lane_width / 2 + (i * lane_width), outer_distance+ (2*lane_width)])
                net.add_lane(
                    "o:"+ str(i) +":"+ str(corner),
                    "ir:"+ str(i) + ":"+  str(corner),
                    StraightLane(
                        start, end, line_types=[c, c], priority=priority, speed_limit=13.4
                    ),
                )
            # Right turn
            r_center = rotation @ (np.array([outer_distance+(2*lane_width), outer_distance+(2*lane_width)]))
            net.add_lane(
                "ir:"+ "2:"+  str(corner),
                "il:"+ "2:" + str((corner - 1) % 4),
                CircularLane(
                    r_center,
                    right_turn_radius,
                    angle + np.radians(180),
                    angle + np.radians(270),
                    line_types=[c, c],
                    priority=priority,
                    speed_limit=13.4,
                ),
            )
            # Left turn
            l_center = rotation @ (
                np.array(
                    [
                        -left_turn_radius + lane_width / 2,
                        left_turn_radius - lane_width / 2,
                    ]
                )
            )
            net.add_lane(
                "ir:"+ "0:"+ str(corner),
                "il:"+ "0:"+  str((corner + 1) % 4),
                CircularLane(
                    l_center,
                    left_turn_radius,
                    angle + np.radians(0),
                    angle + np.radians(-90),
                    clockwise=False,
                    line_types=[c, c],
                    priority=priority - 1,
                    speed_limit=13.4,
                ),
            )
            # Straight
            start = rotation @ np.array([lane_width / 2 + lane_width, outer_distance + (2*lane_width)])
            end = rotation @ np.array([lane_width / 2+ lane_width, -outer_distance - (2*lane_width)])
            net.add_lane(
                "ir:"+ "1:"+str(corner),
                "il:"+ "1:"+ str((corner + 2) % 4),
                StraightLane(
                    start, end, line_types=[c, c], priority=priority, speed_limit=13.4
                ),
            )
            # Exit
            # start = rotation @ np.flip(
            #     [lane_width / 2, access_length + outer_distance], axis=0
            # )
            # end = rotation @ np.flip([lane_width / 2, outer_distance], axis=0)
            # net.add_lane(
            #     "il" + str((corner - 1) % 4),
            #     "o" + str((corner - 1) % 4),
            #     StraightLane(
            #         end, start, line_types=[n, c], priority=priority, speed_limit=10
            #     ),
            # )
            # Exit
            for i in range(3):
                start = rotation @ np.flip(
                    [lane_width / 2 + (i * lane_width), outer_distance + (2*lane_width)]
                )
                end = rotation @ np.flip(
                    [lane_width / 2 + (i * lane_width), access_length + outer_distance + (2*lane_width)]
                )
                net.add_lane(
                    "il:" + str(i) + ":" + str((corner - 1) % 4),  # Source lane identifier
                    "o:" + str(i) + ":" + str((corner - 1) % 4),  # Destination lane identifier
                    StraightLane(
                        start, end, line_types=[c, c], priority=priority, speed_limit=13.4
                    ),
                )

        road = RegulatedRoad(
            network=net,
            np_random=self.np_random,
            record_history=self.config["show_trajectories"],
        )
        self.road = road
    
    def _make_vehicles(self, n_vehicles: int = 10) -> None:
        """
        Populate a road with several vehicles on the highway and on the merging lane

        :return: the ego-vehicle
        """
        # Configure vehicles
        vehicle_type = utils.class_from_path(self.config["other_vehicles_type"])
        vehicle_type.DISTANCE_WANTED = 7  # Low jam distance
        vehicle_type.COMFORT_ACC_MAX = 6
        vehicle_type.COMFORT_ACC_MIN = -3

        # Random vehicles
        simulation_steps = 3
        for t in range(n_vehicles - 1):
            self._spawn_vehicle(np.linspace(0, 80, n_vehicles)[t])
        for _ in range(simulation_steps):
            [
                (
                    self.road.act(),
                    self.road.step(1 / self.config["simulation_frequency"]),
                )
                for _ in range(self.config["simulation_frequency"])
            ]

        # Challenger vehicle
        # self._spawn_vehicle(
        #     60,
        #     spawn_probability=1,
        #     go_straight=True,
        #     position_deviation=0.1,
        #     speed_deviation=0,
        # )

        # Controlled vehicles
        self.controlled_vehicles = []
        print("Controlled vehicles", self.config["controlled_vehicles"])
        for ego_id in range(0, self.config["controlled_vehicles"]):

            # 0 if left, 1 if middle, 2 if right
            right_middle_left = self.np_random.choice(range(3))
            if right_middle_left == 0:  # Left lane
                # Turn left: destination is the road to the left of the current direction
                destination_direction = (ego_id % 4 + 3) % 4  # Counter-clockwise turn
            elif right_middle_left == 2:  # Right lane
                # Turn right: destination is the road to the right of the current direction
                destination_direction = (ego_id % 4 + 1) % 4  # Clockwise turn
            else:  # Middle lane
                # Go straight: destination is the road across the intersection
                destination_direction = (ego_id % 4 + 2) % 4

            ego_lane = self.road.network.get_lane(
                ("o:{}".format(right_middle_left)+":{}".format(ego_id % 4), "ir:{}".format(right_middle_left)+":{}".format(ego_id % 4), 0)
            )
            destination = "o:{}:{}".format(right_middle_left, destination_direction)

            # Assuming 'self.road' has an attribute 'speed_limit' that defines the maximum speed for the road
            max_speed = ego_lane.speed_limit

            # Generate a random speed between 0 and max_speed
            random_speed = self.np_random.uniform(3, 8)

            # Generate a random heading in radians. Assuming full 360-degree freedom, 0 to 2*pi radians.
            random_heading = self.np_random.uniform(0, 2 * np.pi)
            flag=True
            while(flag):
                random_x = self.np_random.uniform(-15, 15)

                random_y = self.np_random.uniform(-15, 15)
                ego_vehicle = self.action_type.vehicle_class(
                    self.road,
                    [random_x,random_y],
                    speed=random_speed,
                    heading=random_heading,
                    controlled=True
                )

                for v in self.road.vehicles:  # Prevent early collisions
                    if not (np.linalg.norm(v.position - ego_vehicle.position) < 3):
                        flag=False
                        


            try:
                ego_vehicle.plan_route_to(destination)
                ego_vehicle.speed_index = ego_vehicle.speed_to_index(
                    ego_lane.speed_limit
                )
                ego_vehicle.target_speed = ego_vehicle.index_to_speed(
                    ego_vehicle.speed_index
                )
            except AttributeError:
                pass

            self.road.vehicles.append(ego_vehicle)
            self.controlled_vehicles.append(ego_vehicle)
            for v in self.road.vehicles:  # Prevent early collisions
                if (
                    v is not ego_vehicle and v not in self.controlled_vehicles
                    and np.linalg.norm(v.position - ego_vehicle.position) < 20
                ):
                    self.road.vehicles.remove(v)

    def _spawn_vehicle(
        self,
        longitudinal: float = 0,
        position_deviation: float = 1.0,
        speed_deviation: float = 1.0,
        spawn_probability: float = 0.6,
        go_straight: bool = False,
    ) -> None:
        if self.np_random.uniform() > spawn_probability:
            return

        right_middle_left = self.np_random.choice(range(3))  # 0: left, 1: middle, 2: right
        route_start = self.np_random.choice(range(4))
        if right_middle_left == 0:  # Intend to turn left
            destination_direction = (route_start + 3) % 4
        elif right_middle_left == 2:  # Intend to turn right
            destination_direction = (route_start + 1) % 4
        else:  # Intend to go straight
            destination_direction = (route_start + 2) % 4 if go_straight else self.np_random.choice(range(4))

        vehicle_type = utils.class_from_path(self.config["other_vehicles_type"])
        vehicle = vehicle_type.make_on_lane(
            self.road,
            ("o:{}:{}".format(right_middle_left, route_start), "ir:{}:{}".format(right_middle_left, route_start), 0),
            longitudinal=(
                longitudinal + 5 + self.np_random.normal() * position_deviation
            ),
            speed=9 + self.np_random.normal() * speed_deviation,
        )
        for v in self.road.vehicles:
            if np.linalg.norm(v.position - vehicle.position) < 15:
                return
        vehicle.plan_route_to("o:{}:{}".format(right_middle_left, destination_direction))
        vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        self.road.vehicles_for_stoppage.append(vehicle)
        return vehicle

    def _clear_vehicles(self) -> None:
        is_leaving = (
            lambda vehicle: vehicle.lane_index[1].startswith("o")
            and vehicle.lane.local_coordinates(vehicle.position)[0] >= vehicle.lane.length - 4 * vehicle.LENGTH
        )
        self.road.vehicles = [
            vehicle
            for vehicle in self.road.vehicles
            if vehicle in self.controlled_vehicles
            or not (is_leaving(vehicle) or vehicle.route is None)
        ]

    def has_arrived(self, vehicle: Vehicle, exit_distance: float = 25) -> bool:
        return (
            vehicle.lane_index[1].startswith("o")
            and vehicle.lane.local_coordinates(vehicle.position)[0] >= exit_distance
        )



class MultiAgentThreeWayIntersectionEnv(ThreeWayIntersectionEnv):
    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config.update(
            {
                "action": {
                    "type": "MultiAgentAction",
                    "action_config": {
                        "type": "DiscreteMetaAction",
                        "lateral": False,
                        "longitudinal": True,
                    },
                },
                "observation": {
                    "type": "MultiAgentObservation",
                    "observation_config": {"type": "Kinematics"},
                },
                "controlled_vehicles": 2,
            }
        )
        return config


class ContinuousIntersectionEnv(ThreeWayIntersectionEnv):
    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config.update(
            {
                "observation": {
                    "type": "Kinematics",
                    "vehicles_count": 5,
                    "features": [
                        "presence",
                        "x",
                        "y",
                        "vx",
                        "vy",
                        "long_off",
                        "lat_off",
                        "ang_off",
                    ],
                },
                "action": {
                    "type": "ContinuousAction",
                    "steering_range": [-np.pi / 3, np.pi / 3],
                    "longitudinal": True,
                    "lateral": True,
                    "dynamical": True,
                },
            }
        )
        return config


TupleMultiAgentIntersectionEnv = MultiAgentWrapper(MultiAgentThreeWayIntersectionEnv)
