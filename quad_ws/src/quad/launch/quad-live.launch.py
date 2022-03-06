import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
       
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
        parameters = [{"frame_parameters_path": frame_parameters_path},{"linked_leg_parameters_path": linked_leg_parameters_path}])
 
    servo_parameters_path = os.path.join(
        get_package_share_directory('quad_motors'),
        'config',
        'servo_parameters.yaml') 

    quad_motors=Node(
        package = 'quad_motors',
        executable = 'quad_motors',
        output='screen',  
        parameters = [{"servo_parameters_path": servo_parameters_path}])    
           
    joy_node=Node(
        package = 'joystick_ros2',
        # name = 'quad_node',
        executable = 'joystick_ros2',
        output='screen')   
    
    ld.add_action(quad_node)    
    ld.add_action(quad_motors)
    ld.add_action(joy_node)
    return ld    
