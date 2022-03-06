'''
    Class interfacing with the bezier generator and kinematics.
'''

from src.kinematics import Kinematics
from src.bezier_gait import BezierGait

class QuadCommander():
    def __init__(self, frame_parameters, linked_leg_parameters):            
        self.bezier_gait = BezierGait(dt=0.01)
        self.kinematics = Kinematics(frame_parameters, linked_leg_parameters)  

    def tick(self, motion_parameters): 
        self.motion_parameters = motion_parameters
        pos = self.motion_parameters.pos
        orn = self.motion_parameters.orn
        StepLength = self.motion_parameters.step_length
        LateralFraction = self.motion_parameters.lateral_fraction
        YawRate = self.motion_parameters.yaw_rate
        StepVelocity = self.motion_parameters.step_velocity
        ClearanceHeight = self.motion_parameters.clearance_height
        PenetrationDepth = self.motion_parameters.penetration_depth       
        contacts = [0, 0, 0, 0] # self.motion_parameters.contacts
        
        # self.bezier_gait.Tswing = self.motion_parameters.swing_period
        # yaw correction TODO  

        # Get feet positions.       
        self.T_bf = self.bezier_gait.GenerateTrajectory(
            StepLength, LateralFraction, YawRate, StepVelocity, self.kinematics.WorldToFoot, ClearanceHeight, PenetrationDepth, contacts)
        
        joint_angles = self.kinematics.inverse_kinematics(orn, pos, self.T_bf)              
        joint_angles_linked_leg = self.kinematics.get_joint_angles_linked_legs(joint_angles)          

        return joint_angles, joint_angles_linked_leg