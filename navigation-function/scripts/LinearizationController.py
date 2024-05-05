import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf2_ros as tf
from tf.transformations import euler_from_quaternion

class LinearizationController:

    def __init__(self):
        self.pose = np.array([])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        
        q_x = odom.pose.pose.orientation.x
        q_y = odom.pose.pose.orientation.y
        q_z = odom.pose.pose.orientation.z
        q_w = odom.pose.pose.orientation.w

        yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
        self.pose = np.array([x, y, yaw])

    def is_goal_reached(self, x_goal, y_goal):
        x = self.pose[0]
        y = self.pose[1]
        tol = 0.2

        d = math.sqrt((x - x_goal)**2 + (y - y_goal)**2)
        if d <= tol:
            return True
        else:
            return False

    def go_to_goal(self, x_goal, y_goal, v_x, v_y):
        d = 0.7
        K_p = 1
        V_max = 0.3
        yaw = self.pose[2]

        u1 = v_x + K_p * (x_goal - self.pose[0])
        u2 = v_y + K_p * (y_goal - self.pose[1])

        V_total = math.sqrt(u1**2 + u2**2)
        if (V_total > V_max):
            u1 = u1 * V_max / V_total
            u2 = u2 * V_max / V_total
    
        A = [[np.cos(yaw), -d * np.sin(yaw)],
            [np.sin(yaw), d * np.cos(yaw)]]
        
        v_w = np.linalg.inv(A) @ [[u1], [u2]]
        v = float(v_w[0])
        w = float(v_w[1])
        
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        
        self.vel_pub.publish(vel)

    def stop_robot(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        
        self.vel_pub.publish(vel)
        