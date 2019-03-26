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
        super(PlaceInSlotState, self).start()

        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)

        self.robot_x = -1
        self.robot_y = -1
        self.robot_theta = -1

        self.forward_dist = 5 #inches
        self.side_angle = 0
        if globals.current_letter % 3 == 0:
            self.side_angle = 15 
        elif globals.current_letter % 3 == 2:
            self.side_angle = -15

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def __drop_off_at_letter__(self):
        #drive to slot
        #t.sleep(1)
        #self.robot_x = -1
        #while (self.robot_x == -1):
        #    pass
        #if self.side_angle != 0:
        #    prev_pose = (self.robot_x, self.robot_y, self.robot_theta)
        #    print('SENDING TURN: {}'.format(self.side_angle))
        #    commands.send_drive_turn_command(self.side_angle)
        #    while prev_pose == (self.robot_x, self.robot_y, self.robot_theta):
        #        pass
        
        self.extra_dist = 0.5 if self.side_angle != 0 else 0
        #prev_pose = (self.robot_x, self.robot_y, self.robot_theta)
        commands.send_drop_block_command(self.forward_dist + self.extra_dist, self.side_angle)
        #while prev_pose == (self.robot_x, self.robot_y, self.robot_theta):
        #    pass

        #drop in slot
        #commands.send_grip_command(commands.CLAW_OPEN)
        #t.sleep(1)

        #back off slot
        #prev_pose = (self.robot_x, self.robot_y, self.robot_theta)
        #commands.send_drive_forward_command(-16)
        #while prev_pose == (self.robot_x, self.robot_y, self.robot_theta):
            #pass

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

        if globals.current_block == globals.num_blocks:
            from return_to_home_state import ReturnToHomeState
            return ReturnToHomeState()
        else:
            from drive_to_block_state import DriveToBlockState 
            return DriveToBlockState()
