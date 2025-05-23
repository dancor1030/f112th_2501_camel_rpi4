
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'f112th_2501_control_teleop_pi'
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    aeb_params = os.path.join(get_package_share_directory(package_name),'config','aebs.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','mux.yaml')

    rplidar_node = Node(
                    package = 'rplidar_ros',
                    executable = 'rplidar_composition',
                    output = 'screen',
                    parameters = [{
                        # 'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate' : 115200,
                        'frame_id': 'laser',
                        'inverted' : False,
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                        # 'scan_mode': 'Sensitivity'
                    }])

    aeb_node = Node(package='f112th_2501_control_teleop_pi',
                     executable='aebs_node',
                     parameters=[aeb_params],)    


    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )
    

    
    # Launch them all!
    return LaunchDescription([
        # joy_node,
        # teleop_node,
        rplidar_node,
        aeb_node,
        # twist_mux_node
        ])

