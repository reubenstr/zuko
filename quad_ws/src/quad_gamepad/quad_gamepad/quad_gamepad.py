'''
Wired and wireless game controller ROS2 wrapper

Gamepad code (Gamepad.py, Controllers.py) extracted from library:  https://github.com/piborg/Gamepad/
Library fix for Ubuntu 64-bit : # https://github.com/piborg/Gamepad/pull/6
Library supports other controllers types.

Setup for Playstation 4 controller. 
Other controllers will need to be remapped, see Controllers.py

'''

from src import Gamepad
import time
import platform
from math import modf

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class GamepadPublisher(Node):

    def __init__(self):
        super().__init__('quad_gamepad')        
        rclpy.logging._root_logger.log("quad_gamepad startup.", LoggingSeverity.INFO)

        self.declare_parameter('joystick_number', 0)        
        self.joystick_number = self.get_parameter('joystick_number').value     
        rclpy.logging._root_logger.log(f"Joystick number: {str(self.joystick_number)}", LoggingSeverity.INFO)
       
        # Joy message
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.header.frame_id = ''
        self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        # Joy node
        self.publish_rate_ms = 10
        self.publisher_ = self.create_publisher(Joy, 'joy', self.publish_rate_ms)       
        self.last_event = None
        self.last_publish_time = 0

    def publish_joy(self):
        current_time = modf(time.time())
        self.joy.header.stamp.sec = int(current_time[1])
        self.joy.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.publisher_.publish(self.joy)
        self.last_publish_time = time.time()    

    def run(self):     
        gamepadType = Gamepad.PS4
        num_axes = 8
        num_buttons = 13
        poll_interval_seconds = self.publish_rate_ms / 1000

        # Wait for a connection      
        while not Gamepad.available(self.joystick_number):
            rclpy.logging._root_logger.log("Gamepad not detected...", LoggingSeverity.WARN)
            time.sleep(1.0)
        
        rclpy.logging._root_logger.log("Gamepad connected.", LoggingSeverity.INFO)
        gamepad = gamepadType(self.joystick_number)              
        gamepad.startBackgroundUpdates()

        # Joystick events handled in the background
        try:
            while gamepad.isConnected() and rclpy.ok():
               
                for i in range (num_axes):
                    self.joy.axes[i] = gamepad.axis(i)  
                
                for i in range (num_buttons):
                    if gamepad.beenPressed(i):                   
                        self.joy.buttons[i] = 1                                             
                    if gamepad.beenReleased(i):                   
                        self.joy.buttons[i] = 0                  
                             
                self.publish_joy()
                time.sleep(poll_interval_seconds)
        except KeyboardInterrupt:
            gamepad.disconnect()
            return
        finally:           
            gamepad.disconnect()    
    

def main(args=None):
    rclpy.init(args=args)    
   
    gamepad_publisher = GamepadPublisher()
    gamepad_publisher.run() 
    
    gamepad_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
