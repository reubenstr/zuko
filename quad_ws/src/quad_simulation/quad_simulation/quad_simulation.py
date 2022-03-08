'''
    Runs simulation as a stand alone node.
    Expects joint_angles.

'''

from time import sleep
import rclpy
from rclpy.node import Node
import numpy as np
import copy
from rclpy.executors import SingleThreadedExecutor
from quad_interfaces.msg import JointAngles
from rclpy.logging import LoggingSeverity

from src.gym_env import GymEnv
from src.gui_param_control import GuiParamControl
from src.env_randomizer import EnvRandomizer

class Commander():
    def __init__(self):
        self.env = GymEnv(render=True,
                                on_rack=True,
                                height_field=False,
                                draw_foot_path=False,
                                env_randomizer=None)
        self.env.reset()
        self.gui_param_controller = GuiParamControl(self.env.spot.quadruped)
        self.action = self.env.action_space.sample()       

    def set_joint_angles(self, joint_angles):
        self.env.pass_joint_angles(joint_angles)

    def tick(self):
        # step simulation
        self.env.step(self.action)


class JointAnglesSubscriber(Node):
    def __init__(self):
        super().__init__('joint_angles_subscriber_node')
        self.subscription = self.create_subscription(JointAngles, 'joint_angles', self.listener_callback, 10)
        self.joint_angles = JointAngles()
        self.flag = False

    def listener_callback(self, msg):
        self.joint_angles = msg.joint_angles   
        self.flag = True    

    def get_joint_angles(self):
        return copy.deepcopy(self.joint_angles)
    
    def is_joint_angle_received(self):
        return self.flag


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging._root_logger.log("QUAD SIMULATION STARTED", LoggingSeverity.INFO)
    
    joint_angles_subscriber = JointAnglesSubscriber()
    executor = SingleThreadedExecutor()   
    executor.add_node(joint_angles_subscriber)

    commander = Commander()

    while rclpy.ok():                  
        if joint_angles_subscriber.is_joint_angle_received():
            commander.set_joint_angles(joint_angles_subscriber.get_joint_angles())        
            commander.tick()
               
        executor.spin_once()

    executor.shutdown()
    joint_angles_subscriber.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
