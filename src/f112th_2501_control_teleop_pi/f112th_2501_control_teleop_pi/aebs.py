import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from icecream import ic
from math import isnan
import time
import numpy as np

class Braking_system(Node):
    def __init__(self):
        super().__init__("aebs_node")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback,10)
        self.emergency_pub = self.create_publisher(Twist,"/cmd_vel_emerg",10)

        self.prevtime = 0.
        self.currtime = 0.
        self.prev_distance = 0.
        self.emergency_msg = Twist()

        ##? LPF 
        self.speed_km1 = 0. # speed in k-1
        self.speed = 0. # speed in time k
        self.alpha = 0.5
        ##? LPF 

        ## CONSTANTS
        self.CONVERT_ms = 1e-3
        self.CONVERT_us = 1e-6
        self.CONVERT_ns = 1e-9
        self.angular_thres = 40
        ## CONSTANTS


        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_time_col', 0.2)
                # ! different param inputs from yaml with defailut
                # ('bool_param', True),
                # ('int_param', 42),
                # ('str_param', 'default'),
                # ('list_param', [0.0, 0.0]),
                # ('nested_param.sub_param1', 0),
                # ('nested_param.sub_param2', 0)
            ]
        )
        self.emergency_msg.linear.x = 0.0
        self.emergency_msg.linear.y = 0.0       

    def lidar_callback(self, data):

        angular_delta = 360/len(data.ranges) #? degrees in between rays
        
        ## CONV FACTOR
        self.CONV_FACTOR = 1/angular_delta
        ## CONV FACTOR

        ## limits definition
        uplim = int((180 + self.angular_thres)*self.CONV_FACTOR)
        dnlim = int((180 - self.angular_thres)*self.CONV_FACTOR)
        frontindex = int(180*self.CONV_FACTOR)
        ## limits definition
        
        ##? SELECT RAY AND COMPUTE TIME_TO_COLLISION
        try:
            rays = [x for x in data.ranges[dnlim:uplim] if x not in (float('inf'), float('-inf'))]
            toprint = min(rays)
            minray = min(rays)

            self.currdistance = data.ranges[frontindex]
            self.currtime = time.time()*1000 #* ms

            dt = (self.currtime - self.prevtime)*self.CONVERT_ms
            close_range = np.round(np.arange(0.09, 0.2, 0.01), 2)
            

            if (round(self.prev_distance, 2) in close_range) and (self.currdistance in (float('inf'), float('-inf')) or isnan(self.currdistance)):
                self.currdistance = 0.1
            else:
                pass
            dx =  (self.currdistance - self.prev_distance)
            self.speed = dx/dt


            filteredspeed = self.lpf(self.speed, self.speed_km1, self.alpha)
            if filteredspeed in (float('inf'), float('-inf')) or isnan(filteredspeed):
                filteredspeed = 10000
            
            self.speed_km1 = filteredspeed

            time_to_collition = abs(minray/filteredspeed)
        except:
            toprint = 'nothing to see'
            filteredspeed = 0
            time_to_collition = 10000000000

        # re-assign vars
        self.prev_distance = self.currdistance
        # self.prevtime = data.header.stamp.nanosec
        self.prevtime = self.currtime
        # re-assign vars

        # print(f'minray = {toprint} | ttc = {time_to_collition} | lpf = {filteredspeed} | speed = {self.speed}')
        print(f'ttc = {time_to_collition} | lpf = {filteredspeed}')
        # print(f'speed = {self.speed}')
        # print(f'{self.currdistance}')
        # print(type(np.arange(0.09, 0.2, 0.01)[1]))
        # print(round(np.arange(0.09, 0.2, 0.01)[1], 2) == 0.10)
        # print(data.ranges[frontindex])
        # print(data.header.stamp)
        # print(dx)
        ##? SELECT RAY AND COMPUTE TIME_TO_COLLISION


        param = self.get_parameter('min_time_col').get_parameter_value().double_value


        if(time_to_collition < param):
            # self.emergency_msg.linear.x = 
            print('!!! BRAKING !!!')
            self.emergency_pub.publish(self.emergency_msg)

    def lpf(self, data, pastdata, alpha):
        filtered = alpha * data + (1 - alpha) * pastdata
        return filtered



def main():
    ic('Im alive')
    rclpy.init()
    braking_system = Braking_system()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
