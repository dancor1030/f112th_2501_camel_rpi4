import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from icecream import ic
import numpy as np
from math import sin, cos, isnan, isinf

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ControlTools import ControllerPID as CPID


class ControlSystem(Node):
    def __init__(self):
        super().__init__("wall_follower_control")
        self.lidar_sub = self.create_subscription(Float32MultiArray, "/car_info", self.lidar_callback,10)
        self.car_pub = self.create_publisher(Twist,"/cmd_vel_cont",10)
        self.horizontal_ray = 0
        self.diagonal_ray = 0
        self.car_params = []
        self.declare_parameters(
        namespace='',
        parameters=[
                ('P', 0.02),
                ('I', 0.0001),                
                ('D', 0.01),
                ('Ts', 0.01), #! CAREFULLY CALCULATE THIS 'Ts' IF YOU ARE GOING TO USE IT
                ('dist_setpoint', 2)
            ]
        )
        P = self.get_parameter('P').value
        I = self.get_parameter('I').value
        D = self.get_parameter('D').value
        Ts = self.get_parameter('Ts').value
        self.dist_setpoint = self.get_parameter('dist_setpoint').value

        #? for DANIEL & ANDY approach ====================================
        # alpha controller
        self.alpha_controller = CPID(P, I, D, Ts)
        # distance to wall controller
        self.dist_controller = CPID(P, I, D, Ts)
        self.dist_controller.setpoint(self.dist_setpoint)
        #? ===============================================================

        # overall controller #? for Rozo's approach ======================
        self.controller = CPID(P, I, D, Ts)
        self.controller.setpoint(0) #! setpoint 0 because we want both 'y' and 'theta' (or alpha) to get to zero
        self.Klin = 0.1
        self.Kang = 1.0
        #? ===============================================================

        self.L = 4 #* forward distance travel (refer to Rozo slides) useful for both approaches

    def lidar_callback(self, data):
        self.alpha = data.data[0]
        self.wall_dist = data.data[1]
        # ic(self.alpha)
        # ic(self.wall_dist) 

        #? Daniel and Andy proposal ============================================================================
        # uses both 'alpha_controller' and 'dist_controller'
        # cd = self.wall_dist*cos(self.alpha) + self.L*sin(self.alpha)
        # alpha_des = self.dist_controller.get_discr_u(cd)
        # self.alpha_controller.setpoint(alpha_des)
        # angvel = self.alpha_controller.get_discr_u(self.alpha)
        # linvel = Klin/(Kang*self.alpha_controller.error[0] + 1)
        #? =====================================================================================================

        #? Rozo's proposal (useful for ackermann's drive) ======================================================
        # uses 'controller' controller

        if isinf(self.wall_dist):
            self.wall_dist = 12
        y_coordinate = self.wall_dist - self.dist_setpoint
        self.measured_var = (y_coordinate + self.L*sin(self.alpha))
        angvel = self.controller.get_discr_u(self.measured_var)
        linvel = self.Klin/(abs(self.Kang*angvel) + 1) # absolute value of angvel because linvel can never be <= 0.
        #? =====================================================================================================
        
        ic(y_coordinate)        
        ic(self.alpha)        
        ic(self.measured_var)
        ic(self.controller.error)


        msg = Twist()
        if isnan(linvel) or isnan(angvel):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = linvel
            msg.angular.z = angvel

        self.car_pub.publish(msg)        


def main():
    ic('Im alive')
    rclpy.init() 
    controlsys = ControlSystem()
    rclpy.spin(controlsys)
    rclpy.shutdown()     


if __name__ == '__main__':
    main()
