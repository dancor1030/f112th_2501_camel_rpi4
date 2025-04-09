import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from icecream import ic
import numpy as np
import time as tm

class PID():
    def __init__(self, kp : float, ki : float, kd : float):
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd

        #! Rozo-like-"CT controller"
        self.__prev_error = 0
        self.__integral = 0
        
    def get_control_action(self, dt : float,  error : float) :
        de = (error - self.__prev_error)/dt
        self.__integral += error*dt
        return error*self.__kp + self.__integral*self.__ki + de*self.__kd
        

class GapFollower(Node):
    def __init__(self):
        super().__init__("gap_follower")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.__lidar_callback,10)
        self.control_action = self.create_publisher(Twist,"/cmd_vel_cont",10)
        self.emergency_action = Twist()
        self.emergency_action.angular.z = 0.0
        self.emergency_action.linear.x = 0.0
        self.Klin = 0.38
        self.Kang = 0.65
        self.prev_time = tm.time()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('P', .63),
                ('I', 0.0001),
                ('D', 0.01),
                ('frontal_rays_number', int(240)), #! The 2 accounts for the 360*2 total rays. # 200 used to work
                ('minimum_object_distance', 1.9),
                ('car_width', 0.22),
                ('corner_minimal_depht', 0.15), #Def at 10c                # ! different param inputs from yaml with default
            ]
        )

        self.P = self.get_parameter('P').get_parameter_value().double_value
        self.I = self.get_parameter('I').get_parameter_value().double_value
        self.D = self.get_parameter('D').get_parameter_value().double_value
        self.frontal_rays_number = self.get_parameter('frontal_rays_number').get_parameter_value().integer_value
        self.minimum_object_distance = self.get_parameter('minimum_object_distance').get_parameter_value().double_value
        self.car_width = self.get_parameter('car_width').get_parameter_value().double_value
        self.corner_minimal_depht = self.get_parameter('corner_minimal_depht').get_parameter_value().double_value
        self.angular_controller = PID(self.P, self.I, self.D)

    def __lidar_callback(self, msg):
        initial_index , visible_rays = self.__get_cone_view(msg)
        gap_relevant_rays = self.__floor_nearest(visible_rays, self.minimum_object_distance)
        interesting_gaps = self.__find_gaps(gap_relevant_rays)
        best_gap = self.__get_best_gap(interesting_gaps, 0.50)
        # ic(best_gap)

        if best_gap == 1020.:
          self.control_action.publish(self.emergency_action)
          self._logger.warning("Entered on emergency mode, no feasable gap found")  
          self.angular_controller.__integral = 0
          self.control_action.publish(self.emergency_action)
          return

        gap_angle_rad, gap_angle_deg = self.__get_gap_angle(best_gap, initial_index)
        ic(gap_angle_deg, gap_angle_rad)
        action = self.__get_control_action(gap_angle_rad)
        self.control_action.publish(action)

    def __get_control_action(self, gap_angle_rad : float) -> Twist:
        error = np.pi - gap_angle_rad
        dt = tm.time() - self.prev_time
        control_action = self.angular_controller.get_control_action(dt, error)
        # ic(control_action)
        action = Twist()
        action.angular.z = -control_action
        linvel = self.Klin/(abs(self.Kang*action.angular.z) + 1)
        # action.linear.x = 1.7
        action.linear.x = linvel
        self.prev_time = tm.time()
        return action

    def __get_gap_angle(self, best_gap: float, init_index : int) -> float:
        aqumulated_sum = 0
        for ray in best_gap:
            aqumulated_sum += ray[1]
        mean_indx = aqumulated_sum//len(best_gap) 
        return (np.pi/360)*(init_index + mean_indx) , (init_index + mean_indx)/2

    def __get_best_gap(self, interesting_gaps : float, k : float) -> float:
        greatest_weight_index = 0
        prev_biggest_weight = 0
        max_ray = 0.
        if len(interesting_gaps) == 0:
            ic("no gap")
            return 1020.

        for ii in range(len(interesting_gaps)):
            for jj in range(len(interesting_gaps[ii])):       
                if interesting_gaps[ii][jj][0] > max_ray:
                    if not np.isinf(interesting_gaps[ii][jj][0]) :
                        max_ray = interesting_gaps[ii][jj][0]
                    
            weight = len(interesting_gaps[ii]) + (1-k)*(max_ray - len(interesting_gaps[ii]))
            if weight > prev_biggest_weight:
                greatest_weight_index = ii
                prev_biggest_weight = weight
            max_ray = 0.
        greatest_weight = interesting_gaps[greatest_weight_index]
        # ic(greatest_weight)
        return greatest_weight

    def __find_gaps(self, relevant_rays : float) -> float:
        corner_rays = self.__find_corners(relevant_rays)
        [self.__supress_rays(corner_ray, relevant_rays) for corner_ray in corner_rays]
        gaps = self.__recognize_gaps(relevant_rays)
        return gaps

    def __recognize_gaps(self, relevant_rays : float) -> float:
        #! This section was made with the help of deepseek, it was late at night, got lazy
        # ! either way, the final result was almost entirely made by me, but i have to be honest :/
        # ! -----------------------------------------------------------------------------
        gaps = []
        current_gap = []

        for ii in range(len(relevant_rays)):
            if relevant_rays[ii] == 0.:
                if len(current_gap) != 0:  
                    gaps.append(current_gap) # if it gets to a 0., it will append eerything before it
                    current_gap = []  #and will with reset the array
            else:
                current_gap.append([relevant_rays[ii], ii]) #Just append the ray

        if current_gap:
            gaps.append(current_gap) #Just in case of the last, if its not empty
        # ! ----------------------------------------------------------------------------
        
        return gaps

    def __supress_rays(self, corner_ray : float, relevant_rays : float) -> float:
        # ! Remember, [index, raylength]
        supression_angle = np.arctan2(self.car_width,corner_ray[1])
        index_supp = supression_angle//(np.pi/180) 
        for ii in range(int(2*index_supp//2 + 1)):
            if (corner_ray[0] - ii) > 0. and (corner_ray[0] - ii) < len(relevant_rays):
                relevant_rays[corner_ray[0] - ii] = 0.        
        # ic(supression_angle,relevant_rays)                

    def __find_corners(self, relevant_rays : float) -> float:
        prev_ray = relevant_rays[0]
        gap_rays = []
        for ii in range(len(relevant_rays)):
            if np.abs(prev_ray - relevant_rays[ii]) > self.corner_minimal_depht:
                gap_rays.append([ii - 1, relevant_rays[ii -1]])
            prev_ray = relevant_rays[ii]
        return gap_rays

    def __get_cone_view(self, msg : LaserScan) -> float:
        return  360 - self.frontal_rays_number//2 , msg.ranges[360 - self.frontal_rays_number//2 : 360 + self.frontal_rays_number//2]

    def __floor_nearest(self, visible_rays : float , thresh : float) -> float:
        return [self.__floor_ray(x, thresh) for x in visible_rays]

    def __floor_ray(self, ray : float, thresh : float) -> float:
        if(ray < thresh):
            return 0.
        elif(np.isinf(ray)):
            return 12.
        else:
            return ray

def main():
    ic('Im alive')
    rclpy.init()
    gapfollower_node = GapFollower()
    rclpy.spin(gapfollower_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
