from  Rosmaster_Lib  import  Rosmaster
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
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

        self.sub = self.create_subscription(Joy, "joy", self.act, 10)
        # self.pub = self.create_publisher()

    def act(self, data):
        # 'remote' means it is the value obtained from the remote controller
        self.remotedir = data.axes[0]
        self.remotethrust = data.axes[5]
        self.reverse = data.buttons[3]

        self.dir = map(self.remotedir, 0, 90, 1, 135)
        if self.reverse == 0:
            self.thrust = map(self.remotethrust, 1, 90, -1, 120)
        else:
            self.thrust = map(self.remotethrust, 1, 90, -1, 75)


        print(f'dir: {self.dir} | thrust: {self.thrust}')
        
        bot.set_pwm_servo(1, self.thrust)
        bot.set_pwm_servo(2, self.dir)


def main(args=None):
    rclpy.init(args=args)
    hw_node = HW_TELEOP()
    rclpy.spin(hw_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()