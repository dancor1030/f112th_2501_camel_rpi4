import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from icecream import ic
import numpy as np

class Distance_finder(Node):
    def __init__(self):
        super().__init__("distance_finder")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.__lidar_callback,10)
        self.car_pub = self.create_publisher(Float32MultiArray,"/car_info",10)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('angle_between_rays', 65)
                ('backwards_index', 0)
                # ! different param inputs from yaml with default
            ]
        )
        self.angle_between_rays = self.get_parameter('angle_between_rays').get_parameter_value().integer_value
        self.backward_angle_index = self.get_parameter('backwards_index').get_parameter_value().integer_value



    def __lidar_callback(self, msg):

        angle_increment = self.angle_between_rays*msg.angle_increment

        data_array = Float32MultiArray()
        
        self.horizontal_ray =  self.__get_horizontal_ray(msg)

        self.diagonal_ray = self.__get_diagonal_ray(msg)

        # ic(self.horizontal_ray , self.diagonal_ray, self.angle_between_rays)

        self.car_params = self.__get_car_params(self.horizontal_ray, self.diagonal_ray,  angle_increment)

        data_array.data = self.car_params

        self.car_pub.publish(data_array) 



    def __get_car_params(self, horizontal_ray : float,  diagonal_ray : float, angle_between : float) -> float:
        alpha = np.arctan2((diagonal_ray*np.cos(angle_between) - horizontal_ray), (diagonal_ray*np.sin(angle_between)))
        # ic(diagonal_ray)
        # ic(self.angle_between_rays)
        distance_to_wall = horizontal_ray*np.cos(alpha)
        return [alpha, distance_to_wall]


    def __get_horizontal_ray(self, msg) -> float:
        self.horizontal_ray = msg.ranges[self.backward_angle_index + len(msg.ranges)//4] #? Rely on the array windback
        return self.horizontal_ray
    
    def __get_diagonal_ray(self, msg) -> float:
        self.diagonal_ray = msg.ranges[self.backward_angle_index + len(msg.ranges)//4 + self.angle_between_rays] #? Rely on the array windback
        return self.diagonal_ray


def main():
    ic('Im alive')
    rclpy.init()
    braking_system = Distance_finder()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
