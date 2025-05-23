import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import threading
from sensor_msgs.msg import Imu
import time
from geometry_msgs.msg import Quaternion
import math
from nav_msgs.msg import Odometry

# from  Rosmaster_Lib  import  Rosmaster

def quaternion_from_euler(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class FrameTransform(Node):
    def __init__(self):
        super().__init__("FrameTransform")
        self.tf_broadcaster = TransformBroadcaster(self) #! From the ros2 example
        self.imu_pub = self.create_publisher(Imu, "Imu_data" ,10)

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.timer_odom = self.create_timer(0.1, self.odom_timer_callback)
        self.timer_lidar = self.create_timer(0.1, self.lidar_timer_callback)
        
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)

        # self.car_robot = Rosmaster()
        
        self.local_accel = [0., 0., 0.]
        self.local_ang_vel = [0., 0., 0.]
        self.local_mag = [0., 0., 0.]
    
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.sigma = [0.0, 0.0]
        self.vel = [0.0, 0.0]
        self.yaw_dot = 0.0

        self.dt = 0.1

        #!----------------Yaw input stuff----------------
        self.sigma_input = 0.0
        self.sigma_offset = 0.0
        self.wheel_base = 0.1


        #* First order system stuff
        self.sigma_tau = 0.1
        self.sigma_alpha = self.dt / (self.sigma_tau + self.dt)
        self.sigma_gain = 1.
        self.sigma_beta = self.sigma_gain*self.sigma_alpha

        #!-----------------------------------------------

        #!----------------Vel input stuff----------------
        self.vel_input = 0.0
        
        self.vel_tau = 0.3
        self.vel_alpha = self.dt / (self.vel_tau + self.dt)
        self.vel_gain = 1.
        self.vel_beta = self.vel_gain*self.vel_alpha
        #!-----------------------------------------------

        print("FrameTransform node started")
        print("vel_alpha: ", self.vel_alpha)
        print("vel_beta: ", self.vel_beta)
        print("sigma_alpha: ", self.sigma_alpha)
        print("sigma_beta: ", self.sigma_beta)

        self.__start_car_state_update()

    def cmd_vel_callback(self, msg):
        self.sigma_input = self.sigma_gain * msg.angular.z  + self.sigma_offset
        self.vel_input = self.vel_gain * msg.linear.x

    def update_car_state(self):
        while(1):
            self.sigma[0] = self.sigma_beta * self.sigma_input + (1 - self.sigma_alpha) * self.sigma[1]
            self.vel[0] = self.vel_beta * self.vel_input + (1 - self.vel_alpha) * self.vel[1]
            self.sigma[1] = self.sigma[0]
            self.vel[1] = self.vel[0]

            self.x += self.vel[0] * self.dt * math.cos(self.theta)
            self.y += self.vel[0] * self.dt * math.sin(self.theta)
            self.yaw_dot = self.vel[0] * self.dt * math.tan(self.sigma[0])/self.wheel_base
            self.theta += self.yaw_dot * self.dt

            print(f"X: {self.x}, Y: {self.y}, Theta: {self.theta}, Velocity: {self.vel[0]}, Yaw_dot: {self.yaw_dot}")
            time.sleep(0.1)

    def __start_car_state_update(self):
        odom_thread = threading.Thread(target=self.update_car_state, args=())
        odom_thread.start()

    def odom_timer_callback(self):

        t = TransformStamped()

        time = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, self.theta)


        t.header.stamp = time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x  
        t.transform.translation.y = self.y 
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        #! --------------- Odom side of history -----------------------------

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity
        odom_msg.twist.twist.linear.x = self.vel[0]
        odom_msg.twist.twist.angular.z = self.yaw_dot

        self.publisher_odom.publish(odom_msg)
        self.tf_broadcaster.sendTransform(t)


    def lidar_timer_callback(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar'

        t.transform.translation.x = 0.05
        t.transform.translation.y = 0.0 
        t.transform.translation.z = 0.1

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


    # def post_imu_data(self):

    #     #! Format given by gemini, revised by me.
    #     imu_msg = Imu()
    #     imu_msg.header.stamp = self.get_clock().now().to_msg()
    #     imu_msg.header.frame_id = 'base_link' #  Important:  IMU frame ID.

    #     self.local_accel[0], self.local_accel[1], self.local_accel[2] = self.car_robot.get_accelerometer_data()
    #     self.local_ang_vel[0], self.local_ang_vel[1], self.local_ang_vel[2] = self.car_robot.get_gyroscope_data()
    #     self.local_mag[0], self.local_mag[1], self.local_mag[2] = self.car_robot.get_magnetometer_data()

    #     # Populate linear acceleration
    #     imu_msg.linear_acceleration.x = self.local_accel[0]
    #     imu_msg.linear_acceleration.y = self.local_accel[1]
    #     imu_msg.linear_acceleration.z = self.local_accel[2]
        
    #     # Populate angular velocity
    #     imu_msg.angular_velocity.x = self.local_ang_vel[0]
    #     imu_msg.angular_velocity.y = self.local_ang_vel[1]
    #     imu_msg.angular_velocity.z = self.local_ang_vel[2]
        

    #     #? Using orientation as a place holder for magnetometer info.
    #     qx, qy, qz, qw = euler_to_quaternion(self.local_mag[0], self.local_mag[1], self.local_mag[2])
    #     imu_msg.orientation.x = qx
    #     imu_msg.orientation.y = qy
    #     imu_msg.orientation.z = qz
    #     imu_msg.orientation.w = qw

    #     imu_msg.angular_velocity_covariance = [0.0] * 9
    #     imu_msg.linear_acceleration_covariance = [0.0] * 9
    #     imu_msg.orientation_covariance = [0.0] * 9
        
    #     self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    frame_node = FrameTransform()
    rclpy.spin(frame_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()