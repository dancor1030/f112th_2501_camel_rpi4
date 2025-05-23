import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import math
import numpy as np
import matplotlib.pyplot as plt
import heapq

expansion_size = 1

def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    data = data * resolution
    return data

def euler_from_quaternion(x, y, z, w):
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

class Nav2501HNode(Node):
    def __init__(self):
        super().__init__('nav_2401_hotel_node')
        self.get_logger().info('nav_2401_hotel_node Started')

        self.subs_map = self.create_subscription(OccupancyGrid, '/map', self.OccGrid_callback, 10)
        self.subs_odom = self.create_subscription(Odometry, '/diff_cont/odom', self.Odom_callback, 10)
        self.subs_goalpose = self.create_subscription(PoseStamped, '/goal_pose', self.Goal_Pose_callback, QoSProfile(depth=10))

        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher_cmdvel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goal_x = []
        self.goal_y = []

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.map_ready = False

    def OccGrid_callback(self, msg):
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = msg.data
        self.map_ready = True

    def Odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

    def Goal_Pose_callback(self, msg):
        self.goal_x.append(msg.pose.position.x)
        self.goal_y.append(msg.pose.position.y)
        wp_ans = input("more way points (y/n): ")
        if wp_ans.lower() == 'n': 
            if self.map_ready:
                self.get_map()
            else:
                self.get_logger().warn('Map not received yet.')

    def get_map(self):
        data = costmap(self.map_data, self.width, self.height, self.resolution)

        column = int((self.x - self.originX) / self.resolution)
        row = int((self.y - self.originY) / self.resolution)

        data[row][column] = 0
        data[data < 0] = 1
        data[data > 5] = 1

        rows, cols = data.shape

        fig, self.ax = plt.subplots(figsize=(8, 8), dpi=100)
        self.ax.grid(True)
        plt.ion()
        plt.show()

        for i in range(0, rows, 5):
            for j in range(0, cols, 5):
                if data[i][j] == 1:
                    self.ax.plot(j, i, 's', color='black', markersize=1)
                else:
                    self.ax.plot(j, i, 's', color='white', markersize=1)

        self.ax.plot(column, row, 'o')
        self.build_path(data, row, column)

    def distance(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def astar(self, array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.distance(start, goal)}
        oheap = [(fscore[start], start)]

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            close_set.add(current)
            for i, j in neighbors:
                neighbor = (current[0] + i, current[1] + j)
                if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                    tentative_g_score = gscore[current] + self.distance(current, neighbor)
                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                        continue
                    if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.distance(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return []

    def build_path(self, data, row, column):
        for i in range(len(self.goal_x)):
            goal = (self.goal_x[i], self.goal_y[i])
            columnH = int((goal[0] - self.originX) / self.resolution)
            rowH = int((goal[1] - self.originY) / self.resolution)

            self.ax.plot(columnH, rowH, 'x')

            path = self.astar(data, (row, column), (rowH, columnH))
            if not path:
                self.get_logger().warn('No path found to goal.')
                continue

            y, x = zip(*path)
            self.ax.plot(x, y, '.', markersize=1)

            path_coords = [(px * self.resolution + self.originX, py * self.resolution + self.originY) for py, px in path]
            self.get_logger().info(f"Path: {path_coords}")

            self.ax.set_aspect('equal')
            plt.gcf().canvas.draw()
            plt.pause(1)

            row, column = rowH, columnH

def main(args=None):
    rclpy.init(args=args)
    node = Nav2501HNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
