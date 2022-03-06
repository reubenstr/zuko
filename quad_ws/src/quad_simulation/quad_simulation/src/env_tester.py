#!/usr/bin/env python

'''
Test the simulation environment.
'''

import numpy as np
import matplotlib.pyplot as plt
import copy
import sys
import time
import os
import argparse

sys.path.append('../')

from gym_env import GymEnv
from gui_param_control import GuiParamControl
from kinematics import Kinematics
from bezier_gait import BezierGait
from env_randomizer import EnvRandomizer

# ARGUMENTS
parser = argparse.ArgumentParser(description="Environment Tester (No Joystick).")
parser.add_argument("-hf",
                    "--HeightField",
                    help="Use HeightField",
                    action='store_true')
parser.add_argument("-r",
                    "--DebugRack",
                    help="Put Spot on an Elevated Rack",
                    action='store_true')
parser.add_argument("-p",
                    "--DebugPath",
                    help="Draw Spot's Foot Path",
                    action='store_true')
parser.add_argument("-ay",
                    "--AutoYaw",
                    help="Automatically Adjust Spot's Yaw",
                    action='store_true')
parser.add_argument("-ar",
                    "--AutoReset",
                    help="Automatically Reset Environment When Spot Falls",
                    action='store_true')
parser.add_argument("-dr",
                    "--DontRandomize",
                    help="Do NOT Randomize State and Environment.",
                    action='store_true')
ARGS = parser.parse_args()


def main():
    """ The main() function. """

    print("STARTING ENVIRONMENT TEST")
    seed = 0
    max_timesteps = 4e6   

    if ARGS.DebugRack:
        on_rack = True
    else:
        on_rack = False

    if ARGS.DebugPath:
        draw_foot_path = True
    else:
        draw_foot_path = False

    if ARGS.HeightField:
        height_field = True
    else:
        height_field = False

    if ARGS.DontRandomize:
        env_randomizer = None
    else:
        env_randomizer = EnvRandomizer()
 
    env = GymEnv(render=True,
                        on_rack=on_rack,
                        height_field=height_field,
                        draw_foot_path=draw_foot_path,
                        env_randomizer=env_randomizer)

    # Set seeds
    env.seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    state = env.reset()

    gui_param_controller = GuiParamControl(env.spot.quadruped)

    spot_model = Kinematics()
    T_bf0 = spot_model.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    bezier_gait = BezierGait(dt=env._time_step)


    action = env.action_space.sample()

    yaw = 0.0

    t = 0
    while t < (int(max_timesteps)):

       
        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod = gui_param_controller.UserInput()

        # experiemental
        # bezier_gait.Tswing = SwingPeriod

        # fix yaw for sim
        yaw = env.return_yaw()
        P_yaw = 5.0
        if ARGS.AutoYaw:
            YawRate += -yaw * P_yaw
        # print("YAW RATE: {}".format(YawRate))

        contacts = state[-4:] 

        # Get Desired Foot Poses
        T_bf = bezier_gait.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf, ClearanceHeight, PenetrationDepth,  contacts)
        
        joint_angles = spot_model.InverseKinimatics(orn, pos, T_bf)

        #FL_Elbow.append(np.degrees(joint_angles[0][-1]))

        # for i, (key, Tbf_in) in enumerate(T_bf.items()):
        #     print("{}: \t Angle: {}".format(key, np.degrees(joint_angles[i])))
        # print("-------------------------")

        env.pass_joint_angles(joint_angles.reshape(-1))
        
        # Get External Observations (data into the simulation for later machine learning)
        # env.spot.GetExternalObservations(bezier_gait, bezier_stepper)
       
        # Step
        # Returns simulation data.
        state, reward, done, _ = env.step(action)


        # print("IMU Roll: {}".format(state[0]))
        # print("IMU Pitch: {}".format(state[1]))
        # print("IMU GX: {}".format(state[2]))
        # print("IMU GY: {}".format(state[3]))
        # print("IMU GZ: {}".format(state[4]))
        # print("IMU AX: {}".format(state[5]))
        # print("IMU AY: {}".format(state[6]))
        # print("IMU AZ: {}".format(state[7]))
        # print("-------------------------")
        if done:
            print("DONE")
            if ARGS.AutoReset:
                env.reset()
                # plt.plot()
                # # plt.plot(FL_phases, label="FL")
                # # plt.plot(FR_phases, label="FR")
                # # plt.plot(BL_phases, label="BL")
                # # plt.plot(BR_phases, label="BR")
                # plt.plot(FL_Elbow, label="FL ELbow (Deg)")
                # plt.xlabel("dt")
                # plt.ylabel("value")
                # plt.title("Leg Phases")
                # plt.legend()
                # plt.show()

        # time.sleep(1.0)

        t += 1
    env.close()
    print(joint_angles)


if __name__ == '__main__':
    main()
