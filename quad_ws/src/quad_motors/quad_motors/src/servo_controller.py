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
            angle = servo_angles[i]

            zero_degrees_pulse_width = self.servo_parameters['zero_degrees_pulse_width'][i]
            pulse_width_per_degree = self.servo_parameters['pulse_width_per_degree'][i]
            invert_direction = self.servo_parameters['invert_direction'][i]
            min_pulse_width = self.servo_parameters['min_pulse_width'][i]
            max_pulse_width = self.servo_parameters['max_pulse_width'][i]
            hardware_pin = self.servo_parameters['map_joint_index_to_driver_pin'][i]

            if invert_direction:
                angle = -angle

                
            pulse_width = pulse_width_per_degree * \
                (angle * (180 / pi)) + zero_degrees_pulse_width

            #if invert_direction:                
            #    corrected_pulse_width = 1500 + (1500 - pulse_width)
            #else:
            #    corrected_pulse_width =  pulse_width
            
            pulse_width = self.clamp(pulse_width, min_pulse_width, max_pulse_width)

            self.servo_driver.set_pulse_width(hardware_pin, int(pulse_width))
