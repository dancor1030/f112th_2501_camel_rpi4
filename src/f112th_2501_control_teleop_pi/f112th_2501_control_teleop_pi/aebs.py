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
        self.speed_km1 = 0. # speed in k-1 (after filtering)
        self.speed = 0. # speed in time k (before filtering)
        self.alpha = 0.8
        ##? LPF 

        ## CONSTANTS
        self.CONVERT_ms = 1e-3
        self.CONVERT_us = 1e-6
        self.CONVERT_ns = 1e-9
        self.angular_thres = 40 #! default = 40
        self.FIRST_TIME = True
        self.close_range = np.round(np.arange(0.09, 0.2, 0.01), 2) #* range for detecting "close objects"
        ## CONSTANTS


        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_time_col', 2.)
            ]
        )
        self.emergency_msg.linear.x = 0.0
        self.emergency_msg.linear.y = 0.0       

    def lidar_callback(self, data):

        ## FIND DELTA ANGLE BETWEEN RAYS
        angular_delta = 360/len(data.ranges) #? degrees in between rays
        ## FIND DELTA ANGLE BETWEEN RAYS
        

        ## CONV FACTOR
        self.CONV_FACTOR = 1/angular_delta
        ## CONV FACTOR


        ## limits definition
        uplim = int((180 + self.angular_thres)*self.CONV_FACTOR)
        dnlim = int((180 - self.angular_thres)*self.CONV_FACTOR)
        frontindex = int(180*self.CONV_FACTOR)
        ## limits definition


        ##? SELECT RAY AND COMPUTE TIME_TO_COLLISION

        ## read rays in between limits EXCLUDING INF & CONVERTING CLOSE VALUES TO 0.1
        try:          
            rays = data.ranges[dnlim:uplim]

            ## CAP MIN DISTANCE SO IT DOES NOT RECOGNIZE IT AS INF
            if self.FIRST_TIME == True:
                self.FIRST_TIME = False
            else:
                for iray in range(len(rays)):
                    if (round(self.prev_rays[iray], 2) in self.close_range) and (rays[iray] in (float('inf'), float('-inf')) or isnan(rays[iray])):
                        rays[iray] = 0.1
                    else:
                        pass
            self.prev_rays = rays
            ## CAP MIN DISTANCE SO IT DOES NOT RECOGNIZE IT AS INF  


            ## EXCLUDE INF VLAUES AND FIND MINIMUM RAY
            rays = [x for x in rays if x not in (float('inf'), float('-inf'))]
            minray = min(rays)
            ## EXCLUDE INF VLAUES AND FIND MINIMUM RAY

        except Exception as e:
            print(e)
            minray = 12
        ## read rays in between limits EXCLUDING INF & CONVERTING CLOSE VALUES TO 0.1


        ## OBTAIN CURRENT DISTANCE
        self.currdistance = data.ranges[frontindex]
        ## OBTAIN CURRENT DISTANCE
 

        ## COMPUTE dt
        self.currtime = time.time()*1000 #* ms
        dt = (self.currtime - self.prevtime)*self.CONVERT_ms #* s
        ## COMPUTE dt


        ## cap FRONT IDEX DISTANCE so it is 0.1 instead of INF
        if (round(self.prev_distance, 2) in self.close_range) and (self.currdistance in (float('inf'), float('-inf')) or isnan(self.currdistance)):
            self.currdistance = 0.1
        else:
            pass
        ## cap FRONT IDEX DISTANCE so it is 0.1 instead of INF


        ## COMPUTE SPEED
        dx =  (self.prev_distance - self.currdistance)
        self.speed = dx/dt
        filteredspeed = self.lpf(self.speed, self.speed_km1, self.alpha)
        ## COMPUTE SPEED


        ## CAP FILTERED SPEED IF NAN OR INF
        if filteredspeed in (float('inf'), float('-inf')) or isnan(filteredspeed):
            filteredspeed = self.speed_km1
        ## CAP FILTERED SPEED IF NAN OR INF


        ## COMPUTE TTC
        if filteredspeed > 0:
            time_to_collition = minray/filteredspeed
        else:
            time_to_collition = 1000 #! BIG SO IT DOES NOT STOP
        ## COMPUTE TTC


        ## re-assign vars
        self.speed_km1 = filteredspeed        
        self.prev_distance = self.currdistance
        self.prevtime = self.currtime
        ## re-assign vars


        #* PRINTS ================================
        # print(f'minray = {toprint} | ttc = {time_to_collition} | lpf = {filteredspeed} | speed = {self.speed}')
        # print(f'ttc = {time_to_collition} | lpf = {filteredspeed}')
        
        print(f'ttc={round(time_to_collition, 3)} | lpf={round(filteredspeed, 3)} | speed={round(self.speed, 3)} | x={round(self.currdistance, 3)} x1={round(self.prev_distance, 3)} dx={round(dx, 3)}')
        
        # print(f'minray = {minray} | filspeed = {filteredspeed} | speed = {self.speed} | dx = {dx}')

        # print(f'speed = {self.speed}')
        # print(f'{self.currdistance}')
        # print(type(np.arange(0.09, 0.2, 0.01)[1]))
        # print(round(np.arange(0.09, 0.2, 0.01)[1], 2) == 0.10)
        # print(data.ranges[frontindex])
        # print(data.header.stamp)
        # print(dx)
        #* PRINTS ================================
        ##? SELECT RAY AND COMPUTE TIME_TO_COLLISION


        param = self.get_parameter('min_time_col').get_parameter_value().double_value


        if(time_to_collition < param):
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
