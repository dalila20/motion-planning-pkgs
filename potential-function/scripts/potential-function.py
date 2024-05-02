import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point as PointMsg
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from LinearizationController import LinearizationController

class PotentialFunction:
    def __init__(self):
        rospy.init_node('potential_function_node', anonymous=False)

        self.pose = np.array([])

        rospy.logwarn_throttle(5, "Waiting for goal...")
        rospy.wait_for_message('/goal', PointMsg)

        # Subscribers
        self.goal_sub = rospy.Subscriber('/goal', PointMsg, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def goal_callback(self, goal):
        self.goal = goal

    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        
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

    def get_attractive_potential(self):
        rospy.wait_for_message('/odom', Odometry)

        # Function parameters
        d_star = 1.0
        zeta = 5

        q = np.array([self.pose[0], self.pose[1]])
        q_goal = np.array([self.goal.x, self.goal.y])

        d_qqgoal = np.linalg.norm(q - q_goal)
        if d_qqgoal <= d_star:
            U_att = 0.5 * zeta * (d_qqgoal ** 2)
            grad_Uatt = zeta * (q - q_goal)
        else:
            U_att = d_star * zeta * d_qqgoal - 0.5 * zeta * (d_star ** 2)
            grad_Uatt = d_star * zeta * (q - q_goal) / d_qqgoal
        
        return U_att, grad_Uatt

    def get_repulsive_potential(self):
        pass

    def run(self):
        controller = LinearizationController()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                grad_attrac = -self.get_attractive_potential()[1]
                controller.go_to_goal(self.goal.x,
                                                self.goal.y,
                                                grad_attrac[0],
                                                grad_attrac[1])

                if (controller.is_goal_reached(self.goal.x,
                                                self.goal.y)):
                    rospy.logwarn("Goal reached!")
                    controller.stop_robot()
                    # self.goal = None se as velocidades zerarem
                    
                rate.sleep()
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pot_function = PotentialFunction()
    print("Potential function initialized!")
    pot_function.run()
