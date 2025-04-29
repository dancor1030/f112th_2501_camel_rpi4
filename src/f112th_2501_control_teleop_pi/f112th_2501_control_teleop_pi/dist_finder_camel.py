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
                ('angle_between_rays', 45),
                ('backwards_index', 0)
                # ! different param inputs from yaml with default
            ]
        )
        self.angle_between_rays = self.get_parameter('angle_between_rays').get_parameter_value().integer_value
        self.backward_angle_index = self.get_parameter('backwards_index').get_parameter_value().integer_value
        self.total_ray_count = 720 #update for different lidar 
        self.sign = 1


    def __lidar_callback(self, msg):

        # angle_increment_bet_rays = self.angle_between_rays*(msg.angle_increment)

        angle_increment_bet_rays = self.angle_between_rays*msg.angle_increment*2

        data_array = Float32MultiArray()
        
        self.horizontal_ray, self.diagonal_ray =  self.__get_rays(msg)

        self.car_params = self.__get_car_params(self.horizontal_ray, self.diagonal_ray, angle_increment_bet_rays)

        data_array.data = self.car_params

        self.car_pub.publish(data_array) 


    def __get_car_params(self, horizontal_ray : float,  diagonal_ray : float, angle_between : float) -> float:
        alpha = np.arctan2((diagonal_ray*np.cos(angle_between) - horizontal_ray), (diagonal_ray*np.sin(angle_between)))
        distance_to_wall = horizontal_ray*np.cos(alpha)
        if self.sign == 1:
            alpha = -alpha
        ic(alpha, distance_to_wall)
        return [alpha, distance_to_wall]


    def __get_rays(self, msg) -> float:
        horizontal_ray_1 = msg.ranges[self.backward_angle_index + len(msg.ranges)//4] #? Rely on the array windback
        horizontal_ray_2 = msg.ranges[self.backward_angle_index + (len(msg.ranges)//4)*3] #? Rely on the array windback
        # ic(horizontal_ray_1,horizontal_ray_2)
        if (horizontal_ray_2 < horizontal_ray_1):
            self.horizontal_ray = horizontal_ray_2
            self.diagonal_ray = msg.ranges[self.backward_angle_index + (len(msg.ranges)//4)*3 - self.angle_between_rays*2] #? Rely on the array windback
            # ic("Izquierda")
            self.sign = 1
        else:
            self.horizontal_ray = horizontal_ray_1
            self.diagonal_ray = msg.ranges[self.backward_angle_index + len(msg.ranges)//4 + self.angle_between_rays*2] #? Rely on the array windback
            # ic("Derecha")
            self.sign = 0

        return self.horizontal_ray , self.diagonal_ray

def main():
    ic('Im alive')
    rclpy.init()
    braking_system = Distance_finder()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()