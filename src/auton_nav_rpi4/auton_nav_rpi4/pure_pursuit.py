import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import math
from icecream import ic



def map(x, xmin, ymin, xmax, ymax):
    a = (ymax - ymin) / (xmax - xmin)
    b = ymin - a*xmin
    y = a*x + b
    return y



class PurePursuit(Node):
    def __init__(self):
        super().__init__('purepursuit')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lookahead_distance', 1.0),
                # ('wheelbase', 0.26)
                ('wheelbase', 0.1)
            ]
        )    

        ## VARIABLES    
        self.LD = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.WB = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_angle = math.radians(45)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #? cmd_pp == command pure pursuit
        ## VARIABLES    

        # Subscriptions
        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer_cmd_vel = self.create_timer(0.1, self.diff_control)

        # Publisher
        self.current_index = 0

        # Variables
        self.waypoints = []
        self.pose = None

    def path_callback(self, msg: Path):
        self.waypoints = msg.poses

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        self.pure_pursuit_control()

    def diff_control(self):
        if not self.waypoints or self.pose is None:
            self.get_logger().info("No pose or waypoints available.")
            return

        # Robot pose
        rx = self.pose.position.x
        ry = self.pose.position.y
        yaw = self.get_yaw_from_quaternion(self.pose.orientation)

        # Find goal point
        goal = None
        i = 0.
        for wp in self.waypoints:
            dx = wp.pose.position.x - rx
            dy = wp.pose.position.y - ry
            dist = math.sqrt(dx**2 + dy**2)
            i += 1.


            if dist >= self.LD and dist <= self.LD +0.1 and i > self.current_index:
                goal = wp.pose
                break

            if dist < self.LD:
                self.current_index = i

        if goal is None:
            self.get_logger().info("No goal found within lookahead distance.")
            return
        
        # Transform goal to robot frame
        dx = goal.position.x - rx
        dy = goal.position.y - ry
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy


        # Avoid division by zero
        if local_x == 0:
            self.get_logger().info("Goal is directly behind the robot.")
            return

        ic(yaw)
    
        angular_z = (math.atan2(local_y,local_x))

        ic(angular_z)

        # Publish command
        drive_msg = Twist()
        drive_msg.linear.x = 0.8  # Constant linear velocity
        drive_msg.angular.z = angular_z
        self.cmd_pub.publish(drive_msg)

    def pure_pursuit_control(self):

        return
        if not self.waypoints or self.pose is None:
            print("No pose")
            return
        
        

        # Robot pose
        rx = self.pose.position.x
        ry = self.pose.position.y
        yaw = self.get_yaw_from_quaternion(self.pose.orientation)

        # Find goal point
        goal = None
        for wp in self.waypoints:
            dx = wp.pose.position.x - rx
            dy = wp.pose.position.y - ry
            # dist = math.hypot(dx, dy)
            dist = math.sqrt(rx**2 + ry**2)
            if dist >= self.LD:
                goal = wp.pose
                break

        if goal is None:
            self.get_logger().info("No goal found within lookahead distance.")
            return

        # Transform goal to robot frame
        dx = goal.position.x - rx
        dy = goal.position.y - ry
        local_x = math.cos(-yaw)*dx - math.sin(-yaw)*dy #! 'x' axis is in the direction of motion (pointing outwards the front of the car)
        local_y = math.sin(-yaw)*dx + math.cos(-yaw)*dy #! 'y' axis is in the lateral direction, located in the rear axle, perpendicular to the longitudinal direction of the car (direction of motion)

        # Avoid division by zero
        if local_x == 0:
            return

        # Pure pursuit steering angle
        curvature = 2 * local_y / (self.LD ** 2)
        steering_angle = math.atan(self.WB * curvature)
        steering_angle = map(steering_angle, 0, 0, self.max_angle, 1)

        # Publish command
        drive_msg = Twist()
        drive_msg.angular.z = steering_angle
        drive_msg.linear.x = 0.2  # You can set this dynamically
        # self.cmd_pub.publish(drive_msg)

    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)



def main():
    ic('Im alive')
    rclpy.init()
    braking_system = PurePursuit()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()        