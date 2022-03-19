'''
    Drives servos motors
    Expected messages: joint_angles.

'''

from time import sleep
import rclpy
from rclpy.node import Node
import numpy as np
import copy
import sys
import yaml
from rclpy.executors import SingleThreadedExecutor
from quad_interfaces.msg import JointAngles
from rclpy.logging import LoggingSeverity
from src.servo_controller import ServoController

class JointAnglesSubscriber(Node):
    def __init__(self):
        super().__init__('joint_angles_subscriber_node')
        self.subscription = self.create_subscription(JointAngles, 'joint_angles', self.listener_callback, 10)
        self.joint_angles = JointAngles()
        self.flag = False

    def listener_callback(self, msg):
        self.joint_angles = msg.joint_angles  
        self.joint_angles_linked_leg = msg.joint_angles_linked_leg  
        self.flag = True 

    def get_joint_angles_linked_leg(self):
        return copy.deepcopy(self.joint_angles_linked_leg)
    
    def is_joint_angle_received(self):
        return self.flag


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging._root_logger.log("QUAD_MOTORS STARTED", LoggingSeverity.INFO)
    
    joint_angles_subscriber = JointAnglesSubscriber()
    executor = SingleThreadedExecutor()   
    executor.add_node(joint_angles_subscriber)

    joint_angles_subscriber.declare_parameters(
        namespace='',
        parameters=[('servo_parameters_path', None)])

    servo_parameters_path = joint_angles_subscriber.get_parameter(
        'servo_parameters_path').get_parameter_value().string_value
   
    rclpy.logging._root_logger.log(
        "servo_parameters_path: " + servo_parameters_path, LoggingSeverity.INFO)
    
    try:       
        with open(servo_parameters_path, 'r') as stream:
            servo_parameters = yaml.safe_load(stream)
    except: 
        rclpy.logging._root_logger.log("Failed to load parameters from file. Please run servo_calibration.py", LoggingSeverity.FATAL)
        sys.exit(1)

    servo_controller = ServoController(1, 0x40, servo_parameters) 

    while rclpy.ok():                  
        if joint_angles_subscriber.is_joint_angle_received():
            servo_controller.set_servo_angles(joint_angles_subscriber.get_joint_angles_linked_leg())
               
        executor.spin_once()

    executor.shutdown()
    joint_angles_subscriber.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
