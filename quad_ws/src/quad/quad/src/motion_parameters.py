""" 
    Data class containing motion parameters to control 
    the quadruped's movement and states.
"""

import numpy as np
import copy
from dataclasses import dataclass
from enum import Enum


class MotionState(Enum):
    POSE = 1
    MOTION = 2

@dataclass
class MotionParameters:
    pos: np.ndarray = np.array([0.0, 0.0, 0.0])
    orn: np.ndarray = np.array([0.0, 0.0, 0.0])
    step_length: float = 0.0
    lateral_fraction: float = 0.0
    yaw_rate: float = 0.0
    step_velocity: float = 0.001
    swing_period: float = 0.200
    clearance_height: float = 0.045
    penetration_depth: float = 0.003
    contacts = [0, 0, 0, 0]
    motion_state = MotionState.POSE

    orn_x_min: float = -np.pi / 8
    orn_x_max: float = np.pi / 8
    orn_y_min: float = -np.pi / 8
    orn_y_max: float = np.pi / 8
    orn_z_min: float = -np.pi / 8
    orn_z_max: float = np.pi / 8

    pos_x_min: float = -0.100
    pos_x_max: float = 0.100
    pos_y_min: float = -0.100
    pos_y_max: float = 0.100
    pos_z_min: float = -.195 + 0.070
    pos_z_max: float = 0 + 0.070

    yaw_rate_min: float = -np.pi / 4
    yaw_rate_max: float = np.pi / 4

    step_length_min: float = -0.100
    step_length_max: float = 0.100
