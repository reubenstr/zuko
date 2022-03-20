from time import sleep
from math import pi
from src.PCA9685Servos import PCA9685Servos


class ServoController():
    def __init__(self, bus_index, device_address, servo_parameters):
        self.servo_parameters = servo_parameters
        self.servo_driver = PCA9685Servos(bus_index, device_address)

    def clamp(self, num, min_val, max_val):
        return (max(min(num, max_val), min_val))

    def set_servo_angles(self, servo_angles):
        for i in range(12): 
            zero_degrees_pulse_width = self.servo_parameters['zero_degrees_pulse_width'][i]
            pulse_width_per_degree = self.servo_parameters['pulse_width_per_degree'][i]
            invert_direction = self.servo_parameters['invert_direction'][i]
            min_degrees = self.servo_parameters['min_degrees'][i]
            max_degrees = self.servo_parameters['max_degrees'][i]
            hardware_pin = self.servo_parameters['map_joint_index_to_driver_pin'][i]

            angle = servo_angles[i] * (180 / pi)

            if invert_direction:
                angle = -angle

            angle = self.clamp(angle, min_degrees, max_degrees)
                
            pulse_width = angle * pulse_width_per_degree + zero_degrees_pulse_width

            self.servo_driver.set_pulse_width(hardware_pin, int(pulse_width))
