from  Rosmaster_Lib  import  Rosmaster
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32
import time

bot = Rosmaster()

def map(x, xmin, ymin, xmax, ymax):
    a = (ymax - ymin) / (xmax - xmin)
    b = ymin - a*xmin
    y = a*x + b
    return y

class HW_TELEOP(Node):
    def __init__(self):
        super().__init__("hardware_teleop_node")

        # self.joy_sub = self.create_subscription(Joy, '/joy', self.toggle_deadman, 10)
        self.cmdvel_sub = self.create_subscription(Twist, "/cmd_vel", self.act, 10)
        self.thrust_pub = self.create_publisher(Float32, "/thrust", 10)
        self.dir = 90
        self.thrust = 90
        self.deadman = 1
        # self.pub = self.create_publisher()

    def act(self, data):
        self.remotedir = data.angular.z
        self.remotethrust = data.linear.x
        # self.reverse = data.buttons[3]
        msg = Float32()
        msg.data = self.remotethrust  
        self.thrust_pub.publish(msg)
        #! 'remote' means it is the value obtained from the remote controller

        if self.deadman == 0:
            print('deadman is OFF')
            ## STOP EVERYTHING
            bot.set_pwm_servo(1, 90)
            bot.set_pwm_servo(2, 90)
            ## STOP EVERYTHING
        else:
            self.dir = map(self.remotedir, 0, 90, 1, 135)
            self.thrust = map(self.remotethrust, 0, 90, 1, 120)

            print(f'dir: {self.dir} | thrust: {self.thrust}')        
            bot.set_pwm_servo(1, self.dir)
            bot.set_pwm_servo(2, self.thrust)

    # def toggle_deadman(self, data):
    #     self.deadman = data.buttons[4]

def main(args=None):
    rclpy.init(args=args)
    hw_node = HW_TELEOP()
    rclpy.spin(hw_node)
    rclpy.shutdown()
    bot.set_pwm_servo(1, 90)
    bot.set_pwm_servo(2, 90)

if __name__ == '__main__':
    main()