# library to move between files and folders in the O.S.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'f112th_2501_control_teleop'
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')


    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )


    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    
    aeb = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )


    # Launch them all!
    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_mux_node
        ])

