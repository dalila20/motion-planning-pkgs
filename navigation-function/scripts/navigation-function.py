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
        rospy.init_node('navigation_function_node', anonymous=False)

        self.pose = np.array([])

        rospy.logwarn_throttle(5, "Waiting for goal...")
        rospy.wait_for_message('/goal', PointMsg)

        self.continuity_indexes = []
        self.laser_readings = LaserScan()
        self.angles = []
        self.transform = np.array([])

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

            self.angles = angles

            T = self.transform

            # Positions referenced to the robot
            if len(ranges) > 0 and len(angles) > 0:

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

            return np.asarray(continuities_positions)

    def get_smallest_distances(self, indexes, ranges, angles):

        smallest_dist_indexes = []
        global_coords = []
        T = self.transform

        for interval in indexes:
            obstacle_indexes = list(range(interval[0],
                                    interval[1] + 1))
            
            smallest_dist = float('inf')
            smallest_dist_index = 0
            for index in obstacle_indexes:
                dist = ranges[index]
                if dist < smallest_dist:
                    smallest_dist = dist
                    smallest_dist_index = index
            
            smallest_dist_indexes.append(smallest_dist_index)
            x = (ranges[smallest_dist_index] + 0.38) * math.cos(angles[smallest_dist_index])
            y = (ranges[smallest_dist_index] + 0.38) * math.sin(angles[smallest_dist_index])

            coord = np.array([x, y, 1])
            global_coord = np.dot(T, coord)[:2]
            global_coords.append(global_coord)

        return global_coords

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

    def get_navigation_function(self):
        k = 20

        q = np.array([self.pose[0], self.pose[1]])
        q_goal = np.array([self.goal.x, self.goal.y])
        q_0 = np.array([0, 0])

        r_0 = 3.0
        r_i = 0.38

        d_q_goal = np.linalg.norm(q - q_goal)
        d_q_0 = np.linalg.norm(q - q_0)

        #### teste
        x = self.pose[0]
        y = self.pose[1]

        x_0 = 0.0
        y_0 = 0.0

        x_goal = self.goal.x
        y_goal = self.goal.y

        d_q_goal_x = abs(x - x_goal)
        d_q_goal_y = abs(y - y_goal)

        d_q_0_x = abs(x - x_0)
        d_q_0_y = abs(y - y_0)

        obstacles_coords = self.get_smallest_distances(self.continuity_indexes,
                                                   self.laser_readings.ranges,
                                                   self.angles)
        
        beta = - (d_q_0)**2 + r_0**2

        #### teste
        beta_x = - (d_q_0_x)**2 + r_0**2
        beta_y = - (d_q_0_y)**2 + r_0**2

        for coord in obstacles_coords:
            q_i = np.array([coord[0], coord[1]])
            d_q_i = np.linalg.norm(q - q_i)
            beta *= beta + d_q_i**2 - r_i**2

            #### teste
            x_i = coord[0]
            y_i = coord[1]

            d_q_i_x = abs(x - x_i) - 0.3
            d_q_i_y = abs(y - y_i) - 0.3

            beta_x *= beta_x + d_q_i_x**2 - r_i**2
            beta_y *= beta_y + d_q_i_y**2 - r_i**2

        phi = (d_q_goal**2) / (d_q_goal**(2*k) + 3*beta)**(1 / k)

        #### teste
        phi_x = (d_q_goal_x**2) / (d_q_goal_x**(2*k) + 10*beta_x)**(1 / k)
        phi_y = (d_q_goal_y**2) / (d_q_goal_y**(2*k) + 10*beta_y)**(1 / k)

        # return phi
        return phi_x, phi_y
        
    def run(self):
        controller = LinearizationController()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                self.get_continuities_positions(self.continuity_indexes,
                                                    self.laser_readings)
                if len(self.angles) > 0:
                    # v = self.get_navigation_function()
                    # v_x = v * math.cos(self.pose[2])
                    # v_y = v * math.sin(self.pose[2])

                    #### teste
                    v_x, v_y = self.get_navigation_function()
                    print(v_x, v_y)
                    controller.go_to_goal(self.goal.x,
                                                    self.goal.y,
                                                    v_x,
                                                    v_y)

                if (controller.is_goal_reached(self.goal.x,
                                                self.goal.y)):
                    rospy.logwarn_throttle(2, "Goal reached!")
                    controller.stop_robot()
                    # self.goal = None se as velocidades zerarem
                    
                rate.sleep()
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pot_function = PotentialFunction()
    print("Potential function initialized!")
    pot_function.run()
