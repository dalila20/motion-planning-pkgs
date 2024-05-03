import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from BugFSM import BugFSM
from geometry_msgs.msg import Twist

class WaitingForGoal:
    def __init__(self):
        self.fsm = BugFSM().instance()
        self.goal = None
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber('/goal', Point, self.goal_callback)

    def goal_callback(self, goal):
        self.goal = goal

    def execute(self):
        while not rospy.is_shutdown():
            if (self.goal is not None):
                print(self.goal)
                self.goal = None

                self.fsm.goal_received()
                break
            
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)

            rospy.logwarn_throttle(5, "Waiting for goal...")
