from state import State
import time
import commands

class WaitState(State):
    def __init__(self):
        super(WaitState, self).__init__("Wait State")

        for i in range(3):
            commands.send_claw_command(commands.CARRY_ANGLE)
            time.sleep(0.25)
            commands.send_claw_command(commands.CLAW_DETERMINE_MOTHERSHIP_ANGLE)
            time.sleep(0.25)

        commands.set_display_state(commands.WAITING)
             
    def run(self):       
        return self
