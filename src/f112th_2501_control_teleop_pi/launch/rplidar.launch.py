from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Definicion del nodo (pkg_name, node_name)
    rplidar_node = Node(
                    package = 'rplidar_ros',
                    executable = 'rplidar_composition',
                    output = 'screen',
                    parameters = [{
                        'serial_port': '/dev/ttyUSB0',
                        'frame_id': 'laser_frame',
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                    }])



    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    return LaunchDescription([
        rplidar_node,
    ])