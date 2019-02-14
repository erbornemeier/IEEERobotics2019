from state_machine.test_state import TestState
from state_machine.state_machine import StateMachine

stateMachine = StateMachine(TestState())

while True:
    stateMachine.run()
    
