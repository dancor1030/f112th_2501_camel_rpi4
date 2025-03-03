import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from icecream import ic


class Braking_system(Node):
    def __init__(self):
        super().__init__("Braking_system")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback,10)
        self.emergency_pub = self.create_publisher(Twist,"/cmd_vel_emerg",10)

        self.prev_time = 0.
        self.prev_distance = 0.
        self.emergency_msg = Twist()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_time_col', 0.2)
                # ! different param inputs from yaml with defailut
            ]
        )
        self.emergency_msg.linear.x = -0.2
        self.emergency_msg.linear.y = 0.0       

    def lidar_callback(self, data):
        # ic(data.ranges)
        dt = (data.header.stamp.nanosec - self.prev_time)*10e-9
        dx =  abs(data.ranges[180] - self.prev_distance)*10
        speed = dx/dt

        time_to_collition = abs(data.ranges[180]/speed)

        param = self.get_parameter('min_time_col').get_parameter_value().double_value

        print(f'ttc: {time_to_collition} | {param} | {param > time_to_collition}')

        # ic(speed)
        # ic("time to collition :", time_to_collition )
        # ic(param > time_to_collition)
        # if(param > time_to_collition and time_to_collition > 0):
        if(param > time_to_collition):
            self.emergency_pub.publish(self.emergency_msg)
            # print('braking!')
        # else:
            # pass            

        self.prev_distance = data.ranges[180]
        self.prev_time = data.header.stamp.nanosec



def main():
    ic('Im alive')
    rclpy.init()
    braking_system = Braking_system()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
