
'''
    Converts joystick axes and button input into motion parameters for robot control.

    Coded to match a Playstation 4 controller schema. 
    Adjustments may be required for other joysticks/controllers.
'''

import copy
import numpy as np
from src.motion_parameters import MotionParameters, MotionState

class JoystickInterpreter():
    def __init__(self):
        self.motion_parameters = MotionParameters()
        self.mode_toggle_button_release_flag = True

    def map(self, n, in_min, in_max, out_min, out_max):
        return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def get_motion_parameters(self, axes, buttons):

        # button: triangle
        if buttons[3] == True and self.mode_toggle_button_release_flag == True:
            self.mode_toggle_button_release_flag = False
            if self.motion_parameters.motion_state == MotionState.MOTION:
                self.motion_parameters.motion_state = MotionState.POSE
            elif self.motion_parameters.motion_state == MotionState.POSE:
                self.motion_parameters.motion_state = MotionState.MOTION
        elif buttons[3] == False:
            self.mode_toggle_button_release_flag = True

        # TODO: fetch parameters from a centalized param file
       
        zLimit = 0.1

        ornLimit = np.pi / 4

        # pos: X, Y, Z coordinates
        # orn: Roll, Pitch, Yaw angles

        if self.motion_parameters.motion_state == MotionState.POSE:

            # pos

            # left analog stick up/down
            self.motion_parameters.orn[1] = self.map(
                axes[1], -1, 1, self.motion_parameters.orn_y_min, self.motion_parameters.orn_y_max)

            # left analog stick left/right
            self.motion_parameters.orn[2] = - self.map(
                axes[0], -1, 1, self.motion_parameters.orn_z_min, self.motion_parameters.orn_z_max)

            # right analog stick left/right
            self.motion_parameters.orn[0] = self.map(
                axes[2], -1, 1, self.motion_parameters.orn_x_min, self.motion_parameters.orn_x_max)

            # right analog stick up/down
            self.motion_parameters.pos[2] = - self.map(
                axes[5], -1, 1, self.motion_parameters.pos_z_min, self.motion_parameters.pos_z_max)

        if self.motion_parameters.motion_state == MotionState.MOTION:

            # left analog stick up/down
            self.motion_parameters.step_length = self.map(
                axes[1], -1, 1, self.motion_parameters.step_length_min, self.motion_parameters.step_length_max)

            # left analog stick left/right
            self.motion_parameters.yaw_rate = self.map(
                axes[0], -1, 1, self.motion_parameters.yaw_rate_min, self.motion_parameters.yaw_rate_max)

            # right analog stick left/right
            #self.motion_parameters.orn[0] = self.map(
            #    axes[2], -1, 1, -ornLimit, ornLimit)

            # right analog stick up/down
            self.motion_parameters.pos[2] = - self.map(
                axes[5], -1, 1, self.motion_parameters.pos_z_min, self.motion_parameters.pos_z_max)

        return copy.deepcopy(self.motion_parameters)
