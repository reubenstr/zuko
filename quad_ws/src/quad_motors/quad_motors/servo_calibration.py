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
              parameters['min_degrees'][i],
              parameters['max_degrees'][i],
              servo_index_to_name.get(i)))
    text_format = white
    print(text_format + "  select * \tselect servo: 0-11, example: select 5")
    print("  * \t\tset the live pulse width : 500-2500, example: 1500")
    print("  zero * \tset pulse width at zero degrees: 500-2500, example: zero 1500")
    print("  ratio * \tset pulses per degree: 0-50, example: ratio 11.15")
    print("  invert \tinvert servo rotation direct, example: invert")
    print("  min * \tset min degrees: -180 to +180, example: min 0")
    print("  max * \tset max degrees: -180 to +180, example: max 45")
    print("  zero_all * \tset all servos to their zero pulse width value")   
    print("  exit\t\texit")
    print()

def main(args=None):

    warn_user()

    # Default parameters (generated when parameters file does not exist).
    # Matches Zuko's frame (v2.2) parameters.
    parameters = {"zero_degrees_pulse_width": [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
                        "pulse_width_per_degree": [11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5],
                        "invert_direction": [False, False, False, False, True, True, False, False, False, False, True, True],
                        "min_degrees": [-30, 0, 0, -30, 0, 0, -30, 0, 0, -30, 0, 0], 
                        "max_degrees": [30, 90, 90, 30, 90, 90, 30, 90, 90, 30, 90, 90],                     
                        "map_joint_index_to_driver_pin": map_joint_index_to_driver_pin}

    motion_servo_parameters_path = "../config/servo_parameters.yaml" 
    servo_pulse_widths = parameters['zero_degrees_pulse_width']
    servo_min_pulse_width = 250
    servo_max_pulse_width = 2750
    min_degrees = -180.0
    max_degrees = 180.0 
    selected_servo = 0
    
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
            if command == "invert":
                 parameters['invert_direction'][selected_servo] = not parameters['invert_direction'][selected_servo]
            if command == "exit":
                exit()
            if command == "zero_all":
                for i in range(12):
                    pulse_width = clamp(parameters['zero_degrees_pulse_width'][i], servo_min_pulse_width, servo_max_pulse_width) 
                    hardware_pin = parameters['map_joint_index_to_driver_pin'][i]
                    servo_driver.set_pulse_width(hardware_pin, pulse_width) 
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
                    parameters['min_degrees'][selected_servo] = clamp(
                        val, min_degrees, max_degrees)  
                elif command == "max":
                    val = float(read_line_split[1])
                    parameters['max_degrees'][selected_servo] = clamp(
                        val, min_degrees, max_degrees)  
            
            save_parameters(motion_servo_parameters_path, parameters) 
       
if __name__ == '__main__':
    main()
