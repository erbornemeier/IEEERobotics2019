class StateMachine:
    def __init__(self, startingState):
        self.currentState = startingState
        self.currentState.start()

    def run(self):
        nextState = self.currentState.run()
        if nextState != self.currentState:
            self.currentState.finish()
            self.currentState = nextState
            self.currentState.start()
