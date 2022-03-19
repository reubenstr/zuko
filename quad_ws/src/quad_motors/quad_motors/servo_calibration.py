#!/usr/bin/env python3
'''
    Standalone servo calibration script.
    Does not require ROS to execute.
    
'''

from time import sleep
import yaml
import sys
import os
from time import sleep 
from src.PCA9685Servos import PCA9685Servos

# Map joint indexes to the expansion board's PCA9685 hardware pinouts.
map_joint_index_to_driver_pin = [8, 9, 10, 11, 12, 13, 14, 15, 3, 2, 1, 0] 


def warn_user():
    os.system('clear')
    print("*** WARNING ***")
    print("Servo calibration may attempt to move the hips and legs beyond their mechanical limits causing over current and physical damage.")
    print("Detach hips and legs to prevent over-current and physical damage.")
    print("")   
    if not input("Are you sure? (y/n): ").lower().strip()[:1] == "y": sys.exit(1)
    os.system('clear')

def clamp(num, min_val, max_val):
    return (max(min(num, max_val), min_val))

def save_parameters(motion_servo_parameters_path, parameters):
    with open(motion_servo_parameters_path, 'w+', encoding='utf8') as outfile:
        yaml.dump(parameters, outfile, default_flow_style=False, allow_unicode=True)

def print_screen(motion_servo_parameters_path, selected_servo, joint_pulse_widths, parameters):

    green="\033[0;32m"        # Green
    white="\033[0;37m"        # White

    servo_index_to_name = {
        0: "front left hip",
        1: "front left upper leg",
        2: "front right lower leg",
        3: "front right hip",
        4: "front right upper leg",
        5: "front left lower leg",
        6: "back left hip",
        7: "back left upper leg",
        8: "back left lower leg",
        9: "back right hip",
        10: "back right upper leg",
        11: "back right lower leg"}

    os.system('clear')   
    print("SERVO [LIVE PULSE WIDTH] : ZERO PULSE WIDTH, PULSE WIDTH PER DEGREE, MIN P.W., MAX P.W., JOINT")
    for i in range(12):
        text_format = green if selected_servo == i else white
        cur_pulse_width = "[{:>4}]".format(
            joint_pulse_widths[i]) if selected_servo == i else ""
        print(text_format + "{:>2} {:>6}: {:>4} {:>4} {:>5} {:>5} {:>5} ({})".format(i,
              cur_pulse_width,
              parameters['zero_degrees_pulse_width'][i],
              parameters['pulse_width_per_degree'][i],
              parameters['invert_direction'][i],
              parameters['min_pulse_width'][i],
              parameters['max_pulse_width'][i],
              servo_index_to_name.get(i)))
    text_format = white
    print(text_format + "  select * \tselect servo: 0-11, example: select 5")
    print("  * \t\tset the live pulse width : 500-2500, example: 1500")
    print("  zero * \tset pulse width at zero degrees: 500-2500, example: zero 1500")
    print("  ratio * \tset pulses per degree: 0-50, example: ratio 11.15")
    print("  invert \tinvert servo rotation direct, example: invert")
    print("  min * \tset min pulse width: 0-50, example: min 1250")
    print("  max * \tset max pulse width: 0-50, example: max 1750")
    print("  save\t\tsaves pulse width and ratio values of selected servo")
    print("  CTRL+C\texit")
    print()

def main(args=None):

    warn_user()
       
    selected_servo = 0
    servo_pulse_widths = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    servo_min_pulse_width = 250
    servo_max_pulse_width = 2750       
    motion_servo_parameters_path = "../config/servo_parameters.yaml"   

    parameters = {"zero_degrees_pulse_width": [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
                    "pulse_width_per_degree": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
                    "invert_direction": [False, False, False, False, False, False, False, False, False, False, False, False],
                    "min_pulse_width": [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000], 
                    "max_pulse_width": [2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000],                     
                    "map_joint_index_to_driver_pin": map_joint_index_to_driver_pin}
    try:       
        with open(motion_servo_parameters_path, 'r') as stream:
            parameters = yaml.safe_load(stream)
    except:
        print("Failed to load parameters from file.")
        print("Default parameters will be generated at path:")
        print(motion_servo_parameters_path)
        print()
        input("Press Enter to continue...")
        save_parameters(motion_servo_parameters_path, parameters)
    try:
        servo_driver = PCA9685Servos(1, 0x40)
    except FileNotFoundError:
        os.system("clear")
        print("Error: I2C not found!")
        exit(1)

    # TODO: read exception is occuring despite device working, might be a timing issue
    #if servo_driver.is_alive() == False:
    #   print("I2C device not responding.")
    #    exit()
    
    while True:
        print_screen(motion_servo_parameters_path, selected_servo, servo_pulse_widths, parameters)

        try:
            read_line = sys.stdin.readline()
        except KeyboardInterrupt:
            exit()

        read_line_split = read_line.split()

        hardware_pin = parameters['map_joint_index_to_driver_pin'][selected_servo]

        if len(read_line_split) > 0:
            command = read_line_split[0]
            if command == "save":
                save_parameters(motion_servo_parameters_path, parameters)
            if command == "invert":
                 parameters['pulse_width_per_degree'][selected_servo] = not parameters['pulse_width_per_degree'][selected_servo]
            if command.isnumeric():
                val = int(read_line_split[0])
                servo_pulse_widths[selected_servo] = clamp(val, servo_min_pulse_width, servo_max_pulse_width)
                servo_driver.set_pulse_width(hardware_pin, servo_pulse_widths[selected_servo]) 
            if len(read_line_split) > 1:
                if command == "select":
                    val = int(read_line_split[1])
                    selected_servo = clamp(val, 0, 11)
                elif command == "zero":
                    val = int(read_line_split[1])
                    parameters['zero_degrees_pulse_width'][selected_servo] = clamp(
                        val, servo_min_pulse_width, servo_max_pulse_width)
                elif command == "ratio":
                    val = float(read_line_split[1])
                    parameters['pulse_width_per_degree'][selected_servo] = clamp(
                        val, 0, 100)     
                elif command == "min":
                    val = float(read_line_split[1])
                    parameters['min_pulse_width'][selected_servo] = clamp(
                        val, servo_min_pulse_width, servo_max_pulse_width)  
                elif command == "max":
                    val = float(read_line_split[1])
                    parameters['max_pulse_width'][selected_servo] = clamp(
                        val, servo_min_pulse_width, servo_max_pulse_width)   
       
if __name__ == '__main__':
    main()
