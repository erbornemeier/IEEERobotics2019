import rospy

class StateMachine:
    def __init__(self, startingState):
        self.currentState = startingState
        self.currentState.start()

    def run(self):
        nextState = self.currentState.run()
        if nextState != self.currentState:
            rospy.loginfo("[State Machine] Transitioning from [" + str(self.currentState) + "] to [" + str(nextState) + "]")

            #self.currentState.finish()
            self.currentState = nextState
            self.currentState.start()
