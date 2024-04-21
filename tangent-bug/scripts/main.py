import rospy
from BugFSM import BugFSM
from states.WaitingForGoal import WaitingForGoal
from states.MovingToGoal import MovingToGoal
from geometry_msgs.msg import Point

if __name__ == "__main__":
    fsm = BugFSM().instance()
    waiting = WaitingForGoal()
    moving = MovingToGoal()

    while not rospy.is_shutdown():

        current_state = fsm.state

        if current_state == 'WaitingForGoal':
            result = waiting.execute()
        elif current_state == 'MovingToGoal':
            result = moving.execute()
        elif current_state == 'FollowingBoundary':
            # state = FollowingBoundary
            # state.execute()
            pass

    