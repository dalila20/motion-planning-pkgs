from transitions import Machine, State
import rospy

# incluir subscribers para saber qual mudança ocasionou cada estado

class BugFSM:
    _instance = None

    def on_enter_WaitingForGoal(self):
        rospy.loginfo("Waiting for goal.")

    def on_exit_WaitingForGoal(self):
        rospy.loginfo("Goal received!")

    def on_enter_MovingToGoal(self):
        rospy.loginfo("Moving to goal...")

    def on_exit_MovingToGoal(self):
        rospy.logwarn("Exiting MovingToGoal!")

    def on_enter_FollowingBoundary(self):
        rospy.loginfo("Following obstacle boundary...")

    def on_exit_FollowingBoundary(self):
        rospy.logerr("Exiting FollowingBoundary")

    def __init__(self):
        rospy.init_node('bug_fsm', anonymous=False)

        self.states = [
            State(name='WaitingForGoal', on_enter=['on_enter_WaitingForGoal'], on_exit=['on_exit_WaitingForGoal']),
            State(name='MovingToGoal', on_enter=['on_enter_MovingToGoal'], on_exit=['on_exit_MovingToGoal']),
            State(name='FollowingBoundary', on_enter=['on_enter_FollowingBoundary'], on_exit=['on_exit_FollowingBoundary'])]
        self.machine = Machine(model=self, states=self.states, initial=self.states[0])

        self.machine.add_transition(trigger='goal_received', source='WaitingForGoal', dest='MovingToGoal')
        self.machine.add_transition(trigger='goal_reached', source='MovingToGoal', dest='WaitingForGoal')
        self.machine.add_transition(trigger='minimum_found', source='MovingToGoal', dest='FollowingBoundary')
        self.machine.add_transition(trigger='goal_reached', source='FollowingBoundary', dest='WaitingForGoal')
        self.machine.add_transition(trigger='path_found', source='FollowingBoundary', dest='MovingToGoal')
        self.machine.add_transition(trigger='path_not_found', source='FollowingBoundary', dest='WaitingForGoal')

    def execute(self):
        pass

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance