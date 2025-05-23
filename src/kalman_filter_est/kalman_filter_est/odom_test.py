import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import threading
from sensor_msgs.msg import Imu
import time
from  Rosmaster_Lib  import  Rosmaster

def euler_to_quaternion(roll, pitch, yaw):
  rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
  quat = rot.as_quat()
  return quat[0], quat[1], quat[2], quat[3]


class FrameTransform(Node):
    def __init__(self):
        super().__init__("FrameTransform")
        self.tf_broadcaster = TransformBroadcaster(self) #! From the ros2 example
        self.imu_pub = self.create_publisher(Imu, "Imu_data" ,10)

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.timer_odom = self.create_timer(0.01, self.odom_timer_callback)
        self.timer_lidar = self.create_timer(0.01, self.lidar_timer_callback)
        self.car_robot = Rosmaster()
        
        self.local_accel = [0., 0., 0.]
        self.local_ang_vel = [0., 0., 0.]
        self.local_mag = [0., 0., 0.]
    
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.sigma = [0.0, 0.0]
        self.vel = [0.0, 0.0]

        #!----------------Yaw input stuff----------------
        self.sigma_input = 0.0
        self.sigma_offset = 0.0
        self.wheel_base = 0.1


        #* First order system stuff
        self.sigma_tau = 0.1
        self.sigma_alpha = self.dt / (self.sigma_tau + self.dt)
        self.sigma_gain = 0.4
        self.sigma_beta = self.sigma_gain*self.sigma_alpha


        #!-----------------------------------------------

        #!----------------Vel input stuff----------------
        self.vel_input = 0.0
        
        self.vel_tau = 0.3
        self.vel_alpha = self.dt / (self.vel_tau + self.dt)
        self.vel_gain = 0.3
        self.vel_beta = self.vel_gain*self.vel_alpha
        #!-----------------------------------------------

        self.last_time = time.time()
        self.dt = 0.0

    def cmd_vel_callback(self, msg):
        self.dt = time.time() - self.last_time
        self.sigma_input = yaw_gain * msg.angular.z  + self.sigma_offset
        self.vel_input = vel_gain * msg.linear.x

    def update_car_state(self,msg):
        self.sigma[0] = self.sigma_beta * self.sigma_input + (1 - self.sigma_alpha) * self.sigma[1]
        self.vel[0] = self.vel_beta * self.vel_input + (1 - self.vel_alpha) * self.vel[1]
        self.sigma[1] = self.sigma[0]
        self.vel[1] = self.vel[0]

        self.x += self.vel[0] * self.dt * math.cos(yaw)
        self.y += self.vel[0] * self.dt * math.sin(yaw)
        self.yaw += self.vel[0] * self.dt * math.tan(self.sigma[0])/self.wheel_base

    def __start_car_state_update(self):
        odom_thread = threading.Thread(target=self.__kf_update, args=())
        odom_thread.start()

    def odom_timer_callback(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0. 
        t.transform.translation.y = 0. 
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

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


    def post_imu_data(self):

        #! Format given by gemini, revised by me.
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link' #  Important:  IMU frame ID.

        self.local_accel[0], self.local_accel[1], self.local_accel[2] = self.car_robot.get_accelerometer_data()
        self.local_ang_vel[0], self.local_ang_vel[1], self.local_ang_vel[2] = self.car_robot.get_gyroscope_data()
        self.local_mag[0], self.local_mag[1], self.local_mag[2] = self.car_robot.get_magnetometer_data()

        # Populate linear acceleration
        imu_msg.linear_acceleration.x = self.local_accel[0]
        imu_msg.linear_acceleration.y = self.local_accel[1]
        imu_msg.linear_acceleration.z = self.local_accel[2]
        
        # Populate angular velocity
        imu_msg.angular_velocity.x = self.local_ang_vel[0]
        imu_msg.angular_velocity.y = self.local_ang_vel[1]
        imu_msg.angular_velocity.z = self.local_ang_vel[2]
        

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