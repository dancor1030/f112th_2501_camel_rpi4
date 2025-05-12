import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from geometry_msgs.msg import PoseStamped , Twist
from rclpy.qos import QoSProfile
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import heapq

expansion_size = 1 # for the wall, how many values of the matrix will be expanded to the right and left

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width) # 2D array reshape from 1D array
    wall = np.where(data == 100) # extract the wall coordinates
    for i in range(-expansion_size,expansion_size+1): #
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data


def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class Nav2401HNode(Node):
    def __init__(self):
        super().__init__('nav_2401_hotel_node')
        self.get_logger().info('nav_2401_hotel_node Started')

        self.subscription = self.create_subscription(OccupancyGrid,'/map',self.OccGrid_callback,10)
        self.subscription = self.create_subscription(Odometry,'/diff_cont/odom',self.Odom_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.Goal_Pose_callback,QoSProfile(depth=10))

        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_x = []
        self.goal_y = []	



    def OccGrid_callback(self,msg):
        #self.get_logger().info('OccupancyGrid Callback')   
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = msg.data
        print(self.resolution,self.originX ,self.originY,self.width,self.height)

    def Odom_callback(self,msg):
        #self.get_logger().info('Odometry Callback')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        #print(self.x, self.y, self.yaw)

    def Goal_Pose_callback(self,msg):
        self.get_logger().info('Goal Pose Callback')
        self.goal_x.append(msg.pose.position.x)
        self.goal_y.append(msg.pose.position.y)
        print(self.goal_x, self.goal_y)

        wp_ans = input("more way points (y/n)")
        if wp_ans == 'n': 
            #self.follow_path()
            self.get_map()
        else:
            pass

    def get_map(self):
            data = costmap(self.map_data,self.width,self.height,self.resolution) 
            #print(data)
            column = int((self.x- self.originX)/self.resolution) #x,y 
            row = int((self.y- self.originY)/self.resolution) #x,y 

            data[row][column] = 0 #
            data[data < 0] = 1 
            data[data > 5] = 1 

                    # Define the size of the matrix
            rows = data.shape[0]
            cols = data.shape[1]
            print(rows,cols)

            # Create a plot
            fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
            ax.grid(True)

            plt.ion() # set interactive mode on 
            plt.show()

            # Plot obstacles and blank spaces
            for i in range(0,rows,5):
                for j in range(0,cols,5):
                    if data[i][j] == 1:
                        ax.plot(j, i, 's', color='black', markersize=1)  # Mark obstacles with black squares
                    else:
                        ax.plot(j, i, 's', color='white', markersize=1)  # Mark blank spaces with white squares and black border

            plt.plot(column,row,'o') # only one set, default # of points

    def distance(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def astar(array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:distance(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data = data + [start]
                data = data[::-1]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + distance(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + distance(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = distance(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                return data
        return False
    
    def build_path(self):
        print(len(self.goal_x))
        for i in range(len(self.goal_x)):

            self.goal = (self.goal_x[i],self.goal_y[i])
            
            columnH = int((self.goal[0]- self.originX)/self.resolution)
            rowH = int((self.goal[1]- self.originY)/self.resolution)

            ax.plot(columnH,rowH,'x') # only one set, default # of points
            
            path = astar(data,(row,column),(rowH,columnH)) 
            
            print(path)
            y,x  = zip(*path)
            ax.plot(x,y,'.',markersize=1)
            

            path = [(p[1]*self.resolution+self.originX,p[0]*self.resolution+self.originY) for p in path] #x,y 
            
            print(path)
        
            ax.set_aspect('equal')
            plt.gcf().canvas.draw() #update display window
            plt.pause(1)
            
            row = rowH
            column = columnH



def main(args=None):
    rclpy.init(args=args)
    node = Nav2401HNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()