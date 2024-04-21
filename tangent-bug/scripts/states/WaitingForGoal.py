import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from BugFSM import BugFSM

class WaitingForGoal:
    def __init__(self):
        self.fsm = BugFSM().instance()
        self.goal = Point()
        self.goal_sub = rospy.Subscriber('/goal', Point, self.goal_callback)

    def goal_callback(self, goal):
        self.goal = goal

    def execute(self):
        while not rospy.is_shutdown():
            if (self.goal.x != 0 or self.goal.y != 0):
                print(self.goal)

                self.fsm.goal_received()
                return self.goal
            
            rospy.logwarn_throttle(5, "Waiting for goal...")
