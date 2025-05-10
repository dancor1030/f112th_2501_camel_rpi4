import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


from kalman_filter_est.kalman_filter_class import KalmanFilter

import numpy as np
import math
import time

from  Rosmaster_Lib  import  Rosmaster
#! Library http://www.yahboom.net/public/upload/upload-html/1689913026/3.%20Install%20Rosmaster%20driver%20library.html

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__("KalmanFilterNode")

        self.odom_publisher = self.create_publisher(Odometry,"kf_odom",10)
        self.tf_broadcaster = TransformBroadcaster(self) #! From the ros2 example
        self.timer_tf = self.create_timer(0.01, self.tf_timer_callback)
        self.timer_imu_data = self.create_timer(0.01, self.__get_imu_data_callback)
        self.car_robot = Rosmaster()

        self.local_accel = [0., 0., 0.]
        self.local_ang_vel = [0., 0., 0.]
        self.local_mag = [0., 0., 0.]

        self.kf_angle = KalmanFilter()
        self.kf_body = KalmanFilter()

        self.x = np.zeros((3,1))
    

    def tf_timer_callback(self):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        x = seconds * math.pi

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x[0] 
        t.transform.translation.y = self.x[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    
    def __get_imu_data_callback(self):
        """
        Obtener los datos de la imu.
        """
        self.local_accel[0], self.local_accel[1], self.local_accel[2] = self.car_robot.get_accelerometer_data()
        self.local_ang_vel[0], self.local_ang_vel[1], self.local_ang_vel[2], = self.car_robot.get_gyroscope_data()
        self.local_mag[0], self.local_mag[1], self.local_mag[2] = self.car_robot.get_magnetometer_data()









def main(args=None):
    rclpy.init(args=args)
    hw_node = KalmanFilterNode()
    rclpy.spin(hw_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()