import rospy
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from BugFSM import BugFSM
import math
import numpy as np
from LinearizationController import LinearizationController

class FollowingBoundary:
    def __init__(self):
        self.fsm = BugFSM().instance()
        self.goal = Point()
        self.is_local_minimum = False

        self.d_x_goal = 0.0
        self.d_x_oi = 0.0
        self.d_oi_goal = 0.0

        self.d_reached = 0.0
        self.d_followed = 0.0
        self.followed_points = None
        self.followed_distances = []

        self.transform = np.array([])

        self.is_goal_blocked = True

        self.continuity_indexes = []
        self.laser_readings = LaserScan()

        # Subscribers
        self.goal_sub = rospy.Subscriber('/goal', Point, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
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

    def goal_callback(self, goal):
        self.goal = goal

    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        self.d_x_goal = math.sqrt(math.pow((x - self.goal.x), 2)
                                  + math.pow((y - self.goal.y), 2))
        
        q_x = odom.pose.pose.orientation.x
        q_y = odom.pose.pose.orientation.y
        q_z = odom.pose.pose.orientation.z
        q_w = odom.pose.pose.orientation.w

        yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
        self.pose = np.array([x, y, yaw])

        self.transform = self.get_transformation_matrix(self.pose)

    def laser_callback(self, readings):
        self.laser_readings = readings
        self.angle_min = readings.angle_min
        self.angle_max = readings.angle_max
        self.angle_increment = readings.angle_increment

        indexes = []
        init = None
        end = 0

        num_readings = len(self.laser_readings.ranges)

        for i in range(num_readings):
            if (self.laser_readings.ranges[i] <= readings.range_max and init is None):
                init = i
            if (self.laser_readings.ranges[i] > readings.range_max and init is not None):
                end = i - 1
                indexes.append([init, end])
                init = None
            elif (i == (num_readings - 1) and init is not None):
                end = i
                indexes.append([init, end])
                init = None

        self.continuity_indexes = indexes

    def get_continuities_positions(self, indexes, readings):
        if (rospy.wait_for_message('/scan', LaserScan)):
            ranges = readings.ranges
            num_angles = len(ranges)
            angles = []
            continuities_positions = []

            # Getting angles
            for i in range(num_angles):
                if range(num_angles)[i] == 0:
                    angles.append(readings.angle_min)
                else:
                    angles.append(angles[i-1] + readings.angle_increment)

            # angles = list(range((readings.angle_min, readings.angle_max, readings.angle_increment)))

            T = self.transform
            # Positions referenced to the robot
            for i in range(len(indexes)):
                init_x = ranges[indexes[i][0]] * math.cos(angles[indexes[i][0]])
                init_y = ranges[indexes[i][0]] * math.sin(angles[indexes[i][0]])

                end_x = ranges[indexes[i][1]] * math.cos(angles[indexes[i][1]])
                end_y = ranges[indexes[i][1]] * math.sin(angles[indexes[i][1]])

                init_vec = np.array([init_x, init_y, 1])
                end_vec = np.array([end_x, end_y, 1])

                # Positions referenced to the global frame
                global_init_vec = np.dot(T, init_vec)[:2]
                global_end_vec = np.dot(T, end_vec)[:2]

                continuities_positions.append(global_init_vec)
                continuities_positions.append(global_end_vec)

            # Verifies if path to the goal is blocked
            self.is_goal_blocked = self.check_if_blocked(angles, readings, indexes)

            return np.asarray(continuities_positions)

    def get_best_oi(self):
        ois = self.get_continuities_positions(self.continuity_indexes,
                                              self.laser_readings)
        least_dist = float('inf')
        heuristic_distances = []
        best_oi = None

        for index, oi in enumerate(ois):
            # trocar por norm
            d_x_oi = math.sqrt(abs(oi[0]-self.pose[0])**2 +
                             abs(oi[1]-self.pose[1])**2)
            d_oi_goal = math.sqrt(abs(oi[0]-self.goal.x)**2 +
                             abs(oi[1]-self.goal.y)**2)
            if (d_x_oi + d_oi_goal) < least_dist:
                least_dist = d_x_oi + d_oi_goal
                best_oi = oi
                self.d_oi_goal = d_x_oi + d_oi_goal
                heuristic_distances.append(least_dist)

            distance_margin = 0.005
            for index, distance in enumerate(heuristic_distances):
                if index > 0 and (abs(heuristic_distances[index] - heuristic_distances[index - 1]) <= distance_margin):
                    self.is_local_minimum = True
            print(least_dist)

        return best_oi

    def check_if_blocked(self, angles, readings, indexes):
        goal_vec = np.array([self.goal.x, self.goal.y, 1])
        T_inv = np.linalg.inv(self.transform)
        local_goal = np.dot(T_inv, goal_vec)[:2]

        goal_x = local_goal[0]
        goal_y = local_goal[1]

        d_to_goal = np.linalg.norm([goal_x, goal_y])
        distance_margin = 0.3

        angle_to_goal = np.arctan2(goal_y, goal_x)

        closest_angle = float('inf')
        goal_index = 0

        for index, angle in enumerate(angles):
            if abs(angle_to_goal - angle) < closest_angle:
                closest_angle = abs(angle_to_goal - angle)
                goal_index = index

        for index_pair in indexes:
            if index_pair[0] <= goal_index <= index_pair[1]:
                if (readings.ranges[goal_index] - distance_margin) <= d_to_goal:
                    return True
                else:
                    return False

    def get_least_distances(self, indexes, ranges, angles):

        least_dist_indexes = []
        global_coords = []
        T = self.transform

        for interval in indexes:
            obstacle_indexes = list(range(interval[0],
                                    interval[1] + 1))
            
            least_dist = float('inf')
            least_dist_index = 0
            for index in obstacle_indexes:
                dist = ranges[index]
                if dist < least_dist:
                    least_dist = dist
                    least_dist_index = index
            
            least_dist_indexes.append(least_dist_index)
            x = ranges[least_dist_index] * math.cos(angles[least_dist_index])
            y = ranges[least_dist_index] * math.sin(angles[least_dist_index])

            coord = np.array([x, y, 1])
            global_coord = np.dot(T, coord)[:2]
            global_coords.append(global_coord)

        return global_coords

    def update_d_followed(self, point):
        self.followed_points.append(point)
        d = np.linalg.norm(self.goal.x - point[0],
                           self.goal.y - point[1])
        
        self.followed_distances.append(d)
        self.d_followed = min(self.followed_distances)

    def update_d_reach(self, point):
        pass


    def execute(self):
        controller = LinearizationController()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                if self.followed_points is None:
                    controller.stop_robot()
                    self.followed_points = []
                    self.followed_points.append([self.pose[0],
                                                 self.pose[1]])
                    self.get_continuities_positions(self.continuity_indexes,
                                                    self.laser_readings)
                    self.W = (self.laser_readings.ranges[88]
                              + self.laser_readings.ranges[89]) / 2
                    print(self.W)
                
                rate.sleep()
            except KeyboardInterrupt:
                # Ctrl + Z
                break