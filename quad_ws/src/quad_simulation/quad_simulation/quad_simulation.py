'''
    Runs simulation as a stand alone node.
    Expects joint_angles.

'''

from time import sleep

from click import command
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
                          on_rack=False,
                          height_field=False,
                          draw_foot_path=False,
                          env_randomizer=None)
        # self.env.reset()
        self.gui_param_controller = GuiParamControl(self.env.spot.quadruped)
        self.action = self.env.action_space.sample()
  
        
    def set_joint_angles(self, joint_angles):
        self.env.pass_joint_angles(joint_angles)

    def tick(self):
        # step simulation
        self.env.step(self.action)

    def update_camera(self):
        self.gui_param_controller.update_camera()

    def check_reset(self):
        if self.gui_param_controller.check_reset_key():
           self.env.reset() 
           
  
class JointAnglesSubscriber(Node):
    def __init__(self):
        super().__init__('joint_angles_subscriber_node')
        self.subscription = self.create_subscription(
            JointAngles, 'joint_angles', self.listener_callback, 10)
        self.joint_angles = np.zeros(12)

    def listener_callback(self, msg):
        self.joint_angles = msg.joint_angles
        self.flag = True

    def get_joint_angles(self):
        return copy.deepcopy(self.joint_angles)


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging._root_logger.log(
        "QUAD_SIMULATION STARTED", LoggingSeverity.INFO)

    joint_angles_subscriber = JointAnglesSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(joint_angles_subscriber)

    commander = Commander()

    while rclpy.ok():
        
        commander.check_reset()
        commander.update_camera()
        commander.set_joint_angles(joint_angles_subscriber.get_joint_angles())
        commander.tick()

        
        sleep(.01)

        executor.spin_once()

    executor.shutdown()
    joint_angles_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
