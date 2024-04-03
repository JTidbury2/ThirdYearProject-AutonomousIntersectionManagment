from abc import ABC
from typing import TYPE_CHECKING, Optional, Sequence, Tuple

import numpy as np


class RoadObject(ABC):

    """
    Common interface for objects that appear on the road.

    For now we assume all objects are rectangular.
    """

    LENGTH: float = 2  # Object length [m]
    WIDTH: float = 2  # Object width [m]

    def __init__(
        self,
        position: Sequence[float],
        heading: float = 0,
        speed: float = 0,
    ):
        """
        :param road: the road instance where the object is placed in
        :param position: cartesian position of object in the surface
        :param heading: the angle from positive direction of horizontal axis
        :param speed: cartesian speed of object in the surface
        """

        self.position = np.array(position, dtype=np.float64)
        self.heading = heading
        self.speed = speed



        # Enable collision with other collidables
        self.collidable = True

        # Collisions have physical effects
        self.solid = True

        # If False, this object will not check its own collisions, but it can still collides with other objects that do
        # check their collisions.
        self.check_collisions = True

        self.diagonal = np.sqrt(self.LENGTH**2 + self.WIDTH**2)
        self.crashed = False
        self.hit = False
        self.impact = np.zeros(self.position.shape)




    # Just added for sake of compatibility
    def to_dict(self, origin_vehicle=None, observe_intentions=True):
        d = {
            "presence": 1,
            "x": self.position[0],
            "y": self.position[1],
            "vx": 0.0,
            "vy": 0.0,
            "cos_h": np.cos(self.heading),
            "sin_h": np.sin(self.heading),
            "cos_d": 0.0,
            "sin_d": 0.0,
        }
        if not observe_intentions:
            d["cos_d"] = d["sin_d"] = 0
        if origin_vehicle:
            origin_dict = origin_vehicle.to_dict()
            for key in ["x", "y", "vx", "vy"]:
                d[key] -= origin_dict[key]
        return d

    @property
    def direction(self) -> np.ndarray:
        return np.array([np.cos(self.heading), np.sin(self.heading)])

    @property
    def velocity(self) -> np.ndarray:
        return self.speed * self.direction










