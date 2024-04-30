import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from sympy import *
import math
import matplotlib.pyplot as plt
from LinearizationController import LinearizationController

class CirclingCurve:
    def __init__(self):

        # Cardioid curve 
        t = np.linspace(0, 2*np.pi, 40)
        x = 2 * np.cos(t) * (1 - np.cos(t))
        y = 2 * np.sin(t) * (1 - np.cos(t))
        self.waypoints = np.column_stack([x, y])

        self.pose = np.array([])

    def get_closest_point(self, pos):
        least_dist = float('inf')

        for index, wp in enumerate(self.waypoints):
            dist = math.sqrt(abs(pos[0]-wp[0])**2 +
                             abs(pos[1]-wp[1])**2)
            if dist < least_dist:
                least_dist = dist
                wp_index = index
        print(wp_index)
        return wp_index

    def get_curve_point(self, angle, time):
        x = self.x.subs([("w", angle), ("t", time)]).evalf()
        y = self.y.subs([("w", angle), ("t", time)]).evalf()
        
        return [x, y]

    def run(self):
        controller = LinearizationController()
        initial_pos = controller.get_current_pos()
        closest_point = self.get_closest_point(initial_pos)
        i = closest_point
        j = 0

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                controller.go_to_goal(self.waypoints[i][0],
                                        self.waypoints[i][1])
                rospy.logwarn(self.waypoints[i])
                if (controller.is_goal_reached(self.waypoints[i][0],
                                            self.waypoints[i][1])):
                    i = i + 1
                if i == len(self.waypoints):
                    i = 0
                    j = j + 1
                rate.sleep()
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    curve = CirclingCurve()
    curve.run()
