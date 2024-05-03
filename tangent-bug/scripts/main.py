import rospy
from BugFSM import BugFSM
from states.WaitingForGoal import WaitingForGoal
from states.MovingToGoal import MovingToGoal
from states.FollowingBoundary import FollowingBoundary
from geometry_msgs.msg import Point

if __name__ == "__main__":
    fsm = BugFSM().instance()
    waiting = WaitingForGoal()
    moving = MovingToGoal()
    following = FollowingBoundary()

    while not rospy.is_shutdown():

        current_state = fsm.state

        if current_state == 'WaitingForGoal':
            waiting.execute()
        elif current_state == 'MovingToGoal':
            moving.execute()
        elif current_state == 'FollowingBoundary':
            following.execute()

    