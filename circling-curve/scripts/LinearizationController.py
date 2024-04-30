import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf2_ros as tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class LinearizationController:

    def __init__(self):
        rospy.init_node('feedback_linearization_controller', anonymous=False)

        self.marker = Marker()
        self.marker.header.frame_id = "base_footprint"
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration()

        self.pose = np.array([])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        
        q_x = odom.pose.pose.orientation.x
        q_y = odom.pose.pose.orientation.y
        q_z = odom.pose.pose.orientation.z
        q_w = odom.pose.pose.orientation.w

        yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
        self.pose = np.array([x, y, yaw])
        self.update_marker()

    def is_goal_reached(self, x_goal, y_goal):
        x = self.pose[0]
        y = self.pose[1]
        tol = 0.2

        d = math.sqrt((x - x_goal)**2 + (y - y_goal)**2)
        if d <= tol:
            return True
        else:
            return False

    def get_velocities(self, x_goal, y_goal, vx=0, vy=0):
        rospy.wait_for_message('/odom', Odometry)
        
        Kp = 0.5
        d = 0.7
        Vmax = 2
        u1 = vx + Kp * (x_goal - self.pose[0])
        u2 = vy + Kp * (y_goal - self.pose[1])
        Vtot = math.sqrt(u1**2 + u2**2)

        if Vtot > Vmax:
            u1 = u1 * Vmax / Vtot
            u2 = u2 * Vmax / Vtot

        A = [
            [np.cos(self.pose[2]), -d * np.sin(self.pose[2])],
            [np.sin(self.pose[2]), d * np.cos(self.pose[2])]
            ]
        vw = np.linalg.inv(A) @ [[u1], [u2]]
        v = float(vw[0])
        w = float(vw[1])
        
        return v, w

    def go_to_goal(self, x_goal, y_goal):
        v, w = self.get_velocities(x_goal, y_goal)
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        
        self.vel_pub.publish(vel)

    def get_current_pos(self):
        rospy.wait_for_message('/odom', Odometry)
        return [self.pose[0], self.pose[1]]

    def update_marker(self):
        point = Point()
        point.x = self.pose[0]
        point.y = self.pose[1]
        point.z = 0.2
        self.marker.points.append(point)

        self.marker_pub.publish(self.marker)