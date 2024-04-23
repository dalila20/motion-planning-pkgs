import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from sympy import *

class CirclingCurve:
    def __init__(self):
        a = 2
        w, t = symbols("w t")
        self.x = a*cos(w*t)*sqrt(2*cos(2*w*t))
        self.y = a*sin(w*t)*sqrt(2*cos(2*w*t))

        print(self.x)

    def get_curve_point(self, angle, time):
        x = self.x.subs([("w", angle), ("t", time)]).evalf()
        y = self.y.subs([("w", angle), ("t", time)]).evalf()
        
        return [x, y]

if __name__ == "__main__":
    curve = CirclingCurve()
    print(curve.get_curve_point(0.1,0.1))
