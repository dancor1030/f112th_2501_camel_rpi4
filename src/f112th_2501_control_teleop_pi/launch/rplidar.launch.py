from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Definicion del nodo (pkg_name, node_name)
    rplidar_node = Node(
                    package = 'rplidar_ros',
                    executable = 'rplidar_composition',
                    output = 'screen',
                    parameters = [{
                        'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                        'frame_id': 'laser_frame',
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                    }])



    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    return LaunchDescription([
        rplidar_node,
    ])