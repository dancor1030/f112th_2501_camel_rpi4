import rclpy
import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node

class HW_TELEOP(Node):
    def __init__(self):
        super().__init__("hardware_teleop_node")

        self.sub = self.create_subscription(Twist, "cmd_vel", self.pwm_response)

    def pwm_response(self, data):
        self.thrust = data.linear.x
        self.angle = data.angular.x

    # def 

def main():
    print('Hi from f112th_2501_control_teleop_pi.')


if __name__ == '__main__':
    main()
