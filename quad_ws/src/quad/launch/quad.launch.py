import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    motion_servo_parameters_path = os.path.join(
        get_package_share_directory('quad'),
        'config',
        'motion_servo_parameters.yaml') 
        
    frame_parameters_path = os.path.join(
        get_package_share_directory('quad'),
        'config',
        'frame_parameters.yaml')    
          
    linked_leg_parameters_path = os.path.join(
        get_package_share_directory('quad'),
        'config',
        'linked_leg_parameters.yaml')  
        
    quad_node=Node(
        package = 'quad',
        #name = 'quad_node',
        executable = 'quad',
        output='screen',  
        parameters = [{"motion_servo_parameters_path": motion_servo_parameters_path}, {"frame_parameters_path": frame_parameters_path},{"linked_leg_parameters_path": linked_leg_parameters_path}])
    
    quad_gamepad=Node(
        package = 'quad_gamepad',
        # name = 'quad_node',
        executable = 'quad_gamepad',
        output='screen',
        parameters=[{"joystick_number": 1}]        
    )      
    ld.add_action(quad_node)
    ld.add_action(quad_gamepad)
    return ld    
