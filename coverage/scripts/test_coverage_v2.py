import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from LinearizationController import LinearizationController
import matplotlib.pyplot as plt
from std_msgs.msg import Bool

class Coverage:
    def __init__(self):
        rospy.init_node('coverage_node', anonymous=False)

        self.pose = np.array([])
        self.obstacles_indexes = []
        self.laser_readings = LaserScan()
        self.transform = np.array([])
        self.angles = np.linspace(-120*math.pi/180, 120*math.pi/180, num=684)
        self.transform = np.array([])

        # Subscribers
        self.pose_sub = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def get_transformation_matrix(self, pose):
        cos_theta = np.cos(pose[2])
        sin_theta = np.sin(pose[2])
    
        R = np.array([[cos_theta, -sin_theta],
                  [sin_theta, cos_theta]])
    
        P = np.array([[pose[0]],
                    [pose[1]]])
    
        T = np.block([[R, P],
                    [0, 0, 1]])
        return T

    def laser_callback(self, readings):
        self.laser_readings = readings
        range_tol = readings.range_max - 0.01

        indexes = []
        init = None
        end = 0

        for i in range(684):
            if (self.laser_readings.ranges[i] < range_tol and init is None):
                init = i
            if (self.laser_readings.ranges[i] >= range_tol and init is not None):
                end = i - 1
                indexes.append([init, end])
                init = None
            elif (i == 683 and init is not None):
                end = i
                indexes.append([init, end])
                init = None

        self.obstacles_indexes = indexes

    def pose_callback(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        
        q_x = pose.pose.orientation.x
        q_y = pose.pose.orientation.y
        q_z = pose.pose.orientation.z
        q_w = pose.pose.orientation.w

        yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
        self.pose = np.array([x, y, yaw])

        self.transform = self.get_transformation_matrix(self.pose)

    def distance(p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) 

    def get_transform_matrix(self):
        cos_theta = np.cos(np.pi)
        sin_theta = np.sin(np.pi)
        
        pos=[-250, 250, 1]

        R = np.array([[1, 0, 0],
                    [0, cos_theta, -sin_theta],
                    [0, sin_theta, cos_theta]])

        P = np.array([[pos[0]],
                    [pos[1]],
                    [pos[2]]])

        T = np.block([[R, P],
                    [0, 0, 0, 1]])
        
        return T

    def polygon_area(self, vertices):
        n = len(vertices)
        area = 0.0
        
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            area += x1 * y2 - y1 * x2

        return abs(area) / 2.0

    def get_grad(self, obstacles_coords, goal):
        # Function parameters
        d_star = 1.0
        zeta = 10
        d_o = 0.2
        eta = 10
        grad_rep = np.array([0.0, 0.0])

        q = np.array([self.pose[0], self.pose[1]])
        q_goal = np.array([goal[0], goal[1]])

        d = q - q_goal
        d_norm = np.linalg.norm(d)
        if d_norm <= d_star:
            grad_att = zeta * d
        else:
            grad_att = d_star * zeta * d / d_norm

        for coord in obstacles_coords:
            q_obs = np.array([coord[0], coord[1]])
            d_obs = q - q_obs
            d_norm = np.linalg.norm(d_obs)

            grad_d_obs = d_obs / d_norm
            
            if d_norm <= d_o:
                grad_rep = grad_rep + eta * (1 / d_o - 1 / d_norm) * grad_d_obs / (d_norm)**2

        # eta = 10
        # q = np.array([self.pose[0], self.pose[1]])

        # q_obs = np.array([coord[0], coord[1]])
        # d_obs = q_obs - q
        # d_norm = np.linalg.norm(d_obs)

        # grad_d_obs = eta * (d_obs / d_norm)
        grad_d_obs = - (grad_att + grad_rep)
        return grad_d_obs
    
    def get_obstacles(self, indexes, readings):
        if (rospy.wait_for_message('/scan', LaserScan)):
            ranges = readings.ranges
            obstacles_positions = []

            T = self.transform

            # Positions referenced to the robot
            if len(indexes) > 0:

                for i in range(len(indexes)):
                    start = indexes[i][0]
                    end = indexes[i][1]
                    interval = list(range(start, end))

                    values = np.array(ranges)[interval]

                    if (values.any()):
                        local_index = np.argmin(values)
                        min_index = interval[local_index]

                        x = ranges[min_index] * math.cos(min_index)
                        y = ranges[min_index] * math.sin(min_index)

                        vec = np.array([x, y, 1])

                        # Positions referenced to the global frame
                        global_vec = np.dot(T, vec)[:2]
                        obstacles_positions.append(global_vec)

            else:
                return np.array([0.0, 0.0])

            return np.asarray(obstacles_positions)

    def get_position_meters(self, pixel):
        T = self.get_transform_matrix()
        vec = np.dot(T, np.array([pixel[0], pixel[1], 1, 1]))[:2]

        point_x = vec[0] * (15 / 500)
        point_y = vec[1] * (15 / 500)

        return [point_x, point_y] 

    def get_cell_safe_border(self, vertices, centroid):
        L = 18 # pixels
        safe_points = []

        for vertex in vertices:
            c = np.array([centroid[0], centroid[1]])
            q = np.array([vertex[0], vertex[1]])

            d = np.linalg.norm(c - q)

            x_goal = vertex[0] + L * (centroid[0] - vertex[0]) / d
            y_goal = vertex[1] + L * (centroid[1] - vertex[1]) / d

            safe_points.append([x_goal, y_goal])
        return safe_points

    def run(self):
        try:
            controller = LinearizationController()
            rate = rospy.Rate(10)

            rospy.wait_for_message('/scan', LaserScan)
            rospy.wait_for_message('/pose', PoseStamped)

            cells = []

            vertices = [[202, 282], [202, 398], [348, 398], [348, 302], [346, 301], [339, 301],
                        [324, 299], [294, 295], [226, 285], [211, 283]] 

            centroid = [272, 344]

            print(self.get_cell_safe_border(vertices, centroid))
            cells.append([vertices, centroid])

            points = [(215.62, 310.93),
                    (284.66, 310.93),
                    (304.62, 310.93),
                    (332.25, 310.93),
                    (215.76, 327.93),
                    (332.49, 327.93),
                    (215.90, 344.93),
                    (332.73, 344.93),
                    (216.04, 361.93),
                    (332.97, 361.93),
                    (216.18, 378.93),
                    (333.21, 378.93)]

            new_points = []

            for i in range(0, len(points), 4):
                if i + 1 < len(points):
                    new_points.append(points[i])
                    new_points.append(points[i + 1])
                if i + 3 < len(points):
                    new_points.append(points[i + 3])
                    new_points.append(points[i + 2])
                elif i + 2 < len(points):
                    new_points.append(points[i + 2])

            print("Vetor modificado:", new_points)
   
            positions = []
            goals = []

            # Go to cell center first
            center = self.get_position_meters(cells[0][1])
            center_x = center[0]
            center_y = center[1]

            while not controller.is_goal_reached(center_x, center_y):
                    q = np.array([self.pose[0], self.pose[1]])
                    positions.append(q)
                    controller.go_to_goal(center_x, center_y, 1.0, 1.0)
                    rospy.sleep(0.1)

                    rospy.loginfo("Going to cell center...")

            for point in new_points:

                goal = self.get_position_meters(point)
                x_goal = goal[0]
                y_goal = goal[1]

                while not controller.is_goal_reached(x_goal, y_goal):

                    obstacles = self.get_obstacles(self.obstacles_indexes,
                                                    self.laser_readings)

                    q = np.array([self.pose[0], self.pose[1]])
                    positions.append(q)

                    grad = self.get_grad(obstacles, [x_goal, y_goal])
                    controller.go_to_goal(x_goal, y_goal, grad[0], grad[1])
                    
                    rospy.sleep(0.1)
                    rospy.loginfo("Going to goal: %f, %f" % (x_goal, y_goal))

                    plt.figure()
                    plt.plot([pos[0] for pos in positions], [pos[1] for pos in positions], label='Trajetória do robô')
                    plt.scatter([goal[0] for goal in goals], [goal[1] for goal in goals], s=3, c='r')        
                    plt.legend()
                    plt.savefig("trajectory.png")
                    plt.close()

                f = open("cells.txt", "w")

                for pos in positions:
                    f.write("%f %f\n" % (pos[0], pos[1]))
                f.close()

        except KeyboardInterrupt:
            return

if __name__ == "__main__":
    cov = Coverage()
    cov.run()