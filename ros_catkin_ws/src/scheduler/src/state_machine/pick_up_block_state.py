import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool

class PickUpBlockState(State):
    def __init__(self):
        super(PickUpBlockState, self).__init__("Pick Up Block")

    def start(self):
        super(PickUpBlockState, self).start()
        #self.cam_pickup_angle = 40

    def run(self):
        commands.send_pickup_command()
        '''
        commands.send_cam_command(self.cam_pickup_angle) 
        t.sleep(0.5)
        commands.send_grip_command(commands.CLAW_CLOSED)
        t.sleep(0.5)
        commands.send_claw_command(commands.PICKUP_ANGLE)
        t.sleep(0.5)
        '''
        from detect_letter_state import DetectLetterState 
        return DetectLetterState()
