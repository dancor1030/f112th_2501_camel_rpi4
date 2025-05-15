
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


from sensor_msgs.msg import Imu
import time
from  Rosmaster_Lib  import  Rosmaster

import math

def euler_to_quaternion(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


class FrameTransform(Node):
    def __init__(self):
        super().__init__("FrameTransform")
        self.tf_broadcaster = TransformBroadcaster(self) #! From the ros2 example

        self.imu_pub = self.create_publisher(Imu, "imu_data" ,10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        self.pose_sub = self.create_subscription(Pose, "robot1/pose", self.pose_sub_callback,10)

        self.timer_odom = self.create_timer(0.01, self.odom_timer_callback)
        self.timer_imu = self.create_timer(0.05, self.post_imu_data)


        self.car_robot = Rosmaster(com="/dev/myserial", debug=True)
        self.car_pose = Pose()
        self.car_robot.create_receive_threading()
        
        time.sleep(.1)
        self.car_robot.set_beep(50)
        time.sleep(.1)

        version = self.car_robot.get_version()
        print("version=", version)

        self.local_accel = [0, 0, 0]
        self.local_ang_vel = [0, 0, 0]
        self.local_mag = [0, 0, 0]
    

    def pose_sub_callback(self, msg : Pose):

        odom_message = Odometry()

        odom_message.header.stamp = self.get_clock().now().to_msg()

        odom_message.child_frame_id = "base_link"

        odom_message.header.frame_id = "odom"

        odom_message.pose.pose = msg

        self.car_pose = msg

        odom_message.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.001, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.001, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.001, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01
        ]

        twist = Twist()
        twist.linear.x = 0.
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.

        odom_message.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.001, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.001, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.001, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01
        ]
        
        odom_message.twist.twist = twist

        self.odom_pub.publish(odom_message)
        

    def odom_timer_callback(self):

        clc = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = clc
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.car_pose.position.x/1000
        t.transform.translation.y = self.car_pose.position.y/1000 
        t.transform.translation.z = self.car_pose.position.z/1000

        t.transform.rotation.x = self.car_pose.orientation.x
        t.transform.rotation.y = self.car_pose.orientation.y
        t.transform.rotation.z = self.car_pose.orientation.z
        t.transform.rotation.w = self.car_pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = clc
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'

        t.transform.translation.x = 0.05
        t.transform.translation.y = 0.0 
        t.transform.translation.z = 0.1

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


    def post_imu_data(self):

        #! Format given by gemini, revised by me.
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link' #  Important:  IMU frame ID.

        print(self.car_robot.get_accelerometer_data())
        
        self.local_accel[0], self.local_accel[1], self.local_accel[2] = self.car_robot.get_accelerometer_data()
        
        self.local_ang_vel[0], self.local_ang_vel[1], self.local_ang_vel[2] = self.car_robot.get_gyroscope_data()

        self.local_mag[0], self.local_mag[1], self.local_mag[2] = self.car_robot.get_magnetometer_data()

        # Populate linear acceleration
        imu_msg.linear_acceleration.x = float(self.local_accel[0])
        imu_msg.linear_acceleration.y = float(self.local_accel[1])
        imu_msg.linear_acceleration.z = float(self.local_accel[2])
        
        # Populate angular velocity
        imu_msg.angular_velocity.x = float(self.local_ang_vel[0])
        imu_msg.angular_velocity.y = float(self.local_ang_vel[1])
        imu_msg.angular_velocity.z = float(self.local_ang_vel[2])
        

        #? Using orientation as a place holder for magnetometer info.
        qx, qy, qz, qw = euler_to_quaternion(self.local_mag[0], self.local_mag[1], self.local_mag[2])
        
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.orientation_covariance = [0.0] * 9
        
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    frame_node = FrameTransform()
    rclpy.spin(frame_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()