import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D

class PlaceInSlotState(State):
    def __init__(self):
        super(PlaceInSlotState, self).__init__("Place in Slot")

    def start(self):
        rospy.loginfo("Place in slot state start")

        self.forward_dist = 3 #inches
        self.side_angle = 0
        if globals.current_letter % 3 == 0:
            self.side_angle =15 
        elif globals.current_letter % 3 == 2:
            self.side_angle = -15

    def __drop_off_at_letter__(self):
        #drive to slot
        t.sleep(0.5)
        if self.side_angle != 0:
            print('SENDING TURN: {}'.format(self.side_angle))
            commands.send_drive_turn_command(self.side_angle)
            t.sleep(5)
        self.extra_dist = 0.5 if self.side_angle != 0 else 0
        commands.send_drive_forward_command(self.forward_dist + self.extra_dist)
        t.sleep(5)

        #drop in slot
        commands.send_grip_command(commands.CLAW_OPEN)
        t.sleep(1)

        #back off slot
        commands.send_drive_forward_command(-2*self.forward_dist)
        t.sleep(5)

        globals.current_block += 1

    def run(self):
        #close claw and lift it up
        t.sleep(0.5) 
        commands.send_grip_command(commands.CLAW_CLOSED)
        t.sleep(0.5)
        commands.send_claw_command(commands.PICKUP_ANGLE)
        t.sleep(0.5)

        #drop it off
        self.__drop_off_at_letter__()
        t.sleep(0.5)

        from drive_to_block_state import * 
        return DriveToBlockState()

    def finish(self):
        rospy.loginfo("Finished pick up block state")
