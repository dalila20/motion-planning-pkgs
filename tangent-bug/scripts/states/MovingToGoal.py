import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from BugFSM import BugFSM
import math
import numpy as np

class MovingToGoal:
    def __init__(self):
        self.fsm = BugFSM().instance()
        self.goal = Point()
        self.local_minimum = Point()

        self.d_x_goal = 0.0
        self.d_x_oi = 0.0
        self.d_oi_goal = 0.0

        self.continuity_indexes = []

        self.angle_min = Float32()
        self.angle_max = Float32()
        self.angle_increment = Float32()

        self.odom = Odometry()
        self.laser_readings = LaserScan()

        # Subscribers
        self.goal_sub = rospy.Subscriber('/goal', Point, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def goal_callback(self, goal):
        self.goal = goal

    def odom_callback(self, odom):
        self.odom = odom
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        self.d_x_goal = math.sqrt(math.pow((x - self.goal.x), 2)
                                  + math.pow((y - self.goal.y), 2))

    def laser_callback(self, readings):
        self.laser_readings = readings
        self.angle_min = readings.angle_min
        self.angle_max = readings.angle_max
        self.angle_increment = readings.angle_increment

        indexes = []
        beginning = 0
        end = 0

        for i in range(len(self.laser_readings.ranges)):
            if (self.laser_readings.ranges[i] <= readings.range_max and beginning == 0):
                beginning = i + 1
            if (self.laser_readings.ranges[i] > readings.range_max and beginning != 0):
                end = i - 1
                indexes.append([beginning, end])
                beginning = 0

        self.continuity_indexes = indexes

    def get_continuities_positions(self, indexes):
        if (rospy.wait_for_message('/scan', LaserScan)):
            num_angles = len(indexes)
            angles = []
            continuities_positions = []
            ranges = self.laser_readings.ranges

            for i in range(num_angles):
                if range(num_angles)[i] == 0:
                    angles.append(self.laser_readings.angle_min)
                else:
                    angles.append(angles[i-1] + self.laser_readings.angle_increment)

            for i in range(num_angles):
                beginning_x = ranges[indexes[i][0]] * math.cos(angles[i])
                beginning_y = ranges[indexes[i][0]] * math.sin(angles[i])

                end_x = ranges[indexes[i][1]] * math.cos(angles[i])
                end_y = ranges[indexes[i][1]] * math.sin(angles[i])

                continuities_positions.append([(beginning_x, beginning_y), (end_x, end_y)])

                # referenciado ao robô?

            return continuities_positions

    def execute(self):
        # if local minimum found
        # self.fsm.minimum_found()
        print(self.get_continuities_positions(self.continuity_indexes))
