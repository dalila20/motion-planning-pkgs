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

class MovingToGoal:
    def __init__(self):
        self.fsm = BugFSM().instance()
        self.goal = Point()
        self.local_minimum = Point()

        self.d_x_goal = 0.0
        self.d_x_oi = 0.0
        self.d_oi_goal = 0.0

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
        init = 0
        end = 0

        for i in range(len(self.laser_readings.ranges)):
            if (self.laser_readings.ranges[i] <= readings.range_max and init == 0):
                init = i + 1
            if (self.laser_readings.ranges[i] > readings.range_max and init != 0):
                end = i - 1
                indexes.append([init, end])
                init = 0
            elif (i == (len(self.laser_readings.ranges)-1) and init != 0):
                # melhorar lógica
                end = i
                indexes.append([init, end])
                init = 0

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
            self.check_if_blocked(angles)

            return np.asarray(continuities_positions)

    def get_best_oi(self):
        ois = self.get_continuities_positions(self.continuity_indexes,
                                              self.laser_readings)
        least_dist = float('inf')
        best_oi = None

        for index, oi in enumerate(ois):
            d_x_oi = math.sqrt(abs(oi[0]-self.pose[0])**2 +
                             abs(oi[1]-self.pose[1])**2)
            d_oi_goal = math.sqrt(abs(oi[0]-self.goal.x)**2 +
                             abs(oi[1]-self.goal.y)**2)
            if (d_x_oi + d_oi_goal) < least_dist:
                least_dist = d_x_oi + d_oi_goal
                best_oi = oi
                self.d_oi_goal = d_x_oi + d_oi_goal
        print(best_oi)

        return best_oi

    def check_if_blocked(self, angles):
        dx = self.goal.x - self.pose[0]
        dy = self.goal.y - self.pose[1]
        d = math.sqrt(dx**2 + dy**2)

        angle_to_target = math.atan2(dy, dx)
        yaw_robot = math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2]))
        angle_diff = angle_to_target - yaw_robot
        index = 0

        least_dist = float('inf')
        for i, angle in enumerate(angles):
            if abs(angle - angle_diff) < least_dist:
                least_dist = abs(angle - angle_diff)
                index = i

        if self.laser_readings.ranges[index] <= self.laser_readings.range_max:
            self.is_goal_blocked = True
        else:
            self.is_goal_blocked = False

        # for pos in [init, end]:
        #     if ((self.goal.x > end[0] and self.goal.y > end[1])
        #         or (self.goal.x < init[0] and self.goal.y < init[1])):
        #         self.is_goal_blocked = False
        #     else:
        #         self.is_goal_blocked = True

    def execute(self):
        # if local minimum found
        # self.fsm.minimum_found()
        controller = LinearizationController()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                oi = self.get_best_oi()

                print(self.is_goal_blocked)
                if self.is_goal_blocked:
                    controller.go_to_goal(oi[0], oi[1])
                    rospy.logwarn(oi)
                else:
                    controller.go_to_goal(self.goal.x,
                                          self.goal.y)
                if (controller.is_goal_reached(self.goal.x,
                                               self.goal.y)):
                    rospy.loginfo("Goal reached!")
                    break
                
                rate.sleep()
            except KeyboardInterrupt:
                # Ctrl + Z
                break
