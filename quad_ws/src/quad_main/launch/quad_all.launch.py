import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    motion_parameters_path = os.path.join(
        get_package_share_directory('quad_main'),
        'config',
        'motion_parameters.yaml') 
        
    frame_parameters_path = os.path.join(
        get_package_share_directory('quad_main'),
        'config',
        'frame_parameters.yaml')    
          
    linked_leg_parameters_path = os.path.join(
        get_package_share_directory('quad_main'),
        'config',
        'linked_leg_parameters.yaml')  
        
    quad_main=Node(
        package = 'quad_main',
        #name = 'quad_main',
        executable = 'quad_main',
        output='screen',  
        parameters = [{"motion_parameters_path": motion_parameters_path}, {"frame_parameters_path": frame_parameters_path},{"linked_leg_parameters_path": linked_leg_parameters_path}])
    
    quad_gamepad=Node(
        package = 'quad_gamepad',
        # name = 'quad_node',
        executable = 'quad_gamepad',
        output='screen',
        parameters=[{"joystick_number": 1}])  
    
    quad_simulation=Node(
        package = 'quad_simulation',
        # name = 'quad_node',
        executable = 'quad_simulation',
        output='screen')    

    ld.add_action(quad_main)
    ld.add_action(quad_gamepad)
    ld.add_action(quad_simulation)
    return ld    
