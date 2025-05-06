
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
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
        self.timer_odom = self.create_timer(0.01, self.odom_timer_callback)
        self.timer_lidar = self.create_timer(0.01, self.lidar_timer_callback)
        self.car_robot = Rosmaster()
        
        self.local_accel = [0., 0., 0.]
        self.local_ang_vel = [0., 0., 0.]
        self.local_mag = [0., 0., 0.]
    
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