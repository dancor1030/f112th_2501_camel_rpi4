import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_name = 'f112th_2501_control_teleop_pi'
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    aeb_params = os.path.join(get_package_share_directory(package_name),'config','aebs.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')


    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    wall_follower_node = Node(package='f112th_2501_control_teleop_pi',
                     executable='wall_follower_node'
                     )    


    distfinder_node = Node(package='f112th_2501_control_teleop_pi',
                     executable='distfinder_node'
                     ) 


    hardware = Node(package='f112th_2501_control_teleop_pi',
                     executable='hardware_teleop'
                     )  

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('cmd_vel','cmd_vel_joy')]
    )

    aeb_node = Node(package='f112th_2501_control_teleop_pi',
                     executable='aebs_node'
                     )  

    # Launch them all!
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource('src/rplidar_ros/launch/rplidar_a1_launch.py')
        # ),
        twist_mux_node,
        hardware,
        teleop_node,
        aeb_node,
        wall_follower_node,
        distfinder_node
        ])
