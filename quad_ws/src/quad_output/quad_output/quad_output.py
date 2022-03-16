'''
    Hardware interface to Zuko's expansion-board's outputs:
        7 LEDs (4 on CONTACT header, and 3 on LED header)
        1 Neopixel on NEO header
'''

from time import sleep
from click import command
import rclpy
from rclpy.node import Node
import numpy as np
import copy
from rclpy.executors import SingleThreadedExecutor
from quad_interfaces.msg import Outputs
from rclpy.logging import LoggingSeverity

  
class OutputsSubscriber(Node):
    def __init__(self):
        super().__init__('outputs_subscriber_node')
        self.subscription = self.create_subscription(
            Outputs, 'outputs', self.listener_callback, 10)
        self.led_states = np.zeros(7)
        self.neopixel_values = []

    def listener_callback(self, msg):
        self.led_states = msg.led_states
        self.neopixel_values = msg.neopixel_values        

    def get_led_states(self):
        return copy.deepcopy(self.led_states)

    def get_neo_pixel_values(self):
        return copy.deepcopy(self.neopixel_values)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging._root_logger.log(
        "QUAD_OUTPUT STARTED", LoggingSeverity.INFO)

    outputs_subscriber = OutputsSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(outputs_subscriber)
   

    while rclpy.ok():

        # DO CODE
             
        sleep(.01)
        executor.spin_once()

    executor.shutdown()
    outputs_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
