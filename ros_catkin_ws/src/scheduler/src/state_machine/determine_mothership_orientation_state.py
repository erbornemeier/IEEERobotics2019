import globals
from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t
from math import *
from geometry_msgs.msg import Pose2D
import drive_utils

class DetermineMothershipOrientationState(State):
    def __init__(self):
        super(DetermineMothershipOrientationState, self).__init__("Determine Mothership Orientation")

    def start(self):
        super(DetermineMothershipOrientationState, self).start()
        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)
        commands.send_cam_command(commands.CAMERA_DETERMINE_MOTHERSHIP_ANGLE)
        t.sleep(1)
        commands.send_claw_command(commands.CLAW_DETERMINE_MOTHERSHIP_ANGLE)
        t.sleep(1)
        commands.send_drive_forward_command(5)
        t.sleep(2)

        self.robot_theta = -1
        self.robot_x = -1
        self.robot_y = -1

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def run(self):
        t.sleep(1)
        try:
            # detected_slot = commands.slot_srv()
            detected_slot = Letter()
            detected_slot.letter = 0

            commands.display_letter(1 if detected_slot.letter == 0 else 4)
            commands.set_display_state(commands.LETTER)

            if detected_slot != 0xFF:
                isABC = detected_slot.letter == 0

                while self.robot_theta == -1:
                    pass


                # TODO: What if robot is turned? How to determine orientation?
                globals.mothership_theta = self.robot_theta if isABC else 180 - self.robot_theta #TODO Check math for DEF, bound angle
                globals.mothership_x = self.robot_x + 11.75 * cos(radians(self.robot_theta))
                globals.mothership_y = self.robot_y + 11.75 * sin(radians(self.robot_theta))

                # If side is ABC diamond point is behind the robot
                # Otherwise it is in front
                mult = 1 if isABC else -1
                diag_width = 40
                diag_width_ramp = 48
                globals.abc_x = globals.mothership_x - (mult * diag_width/2 * cos(radians(self.robot_theta)))
                globals.abc_y = globals.mothership_y - (mult * diag_width/2 * sin(radians(self.robot_theta)))
                globals.def_x = globals.mothership_x + (mult * diag_width/2 * cos(radians(self.robot_theta)))
                globals.def_y = globals.mothership_y + (mult * diag_width/2 * sin(radians(self.robot_theta)))
                
                globals.cd_x = globals.mothership_x - (mult * diag_width_ramp/2 * cos(radians(self.robot_theta + 90)))
                globals.cd_y = globals.mothership_y - (mult * diag_width_ramp/2 * sin(radians(self.robot_theta + 90)))
                globals.af_x = globals.mothership_x - (mult * diag_width_ramp/2 * cos(radians(self.robot_theta - 90)))
                globals.af_y = globals.mothership_y - (mult * diag_width_ramp/2 * sin(radians(self.robot_theta - 90)))

                globals.abc_bb_x = globals.mothership_x - (mult * (diag_width/2 - 1) * cos(radians(self.robot_theta)))
                globals.abc_bb_y = globals.mothership_y - (mult * (diag_width/2 - 1) * sin(radians(self.robot_theta)))
                globals.def_bb_x = globals.mothership_x + (mult * (diag_width/2 - 1) * cos(radians(self.robot_theta)))
                globals.def_bb_y = globals.mothership_y + (mult * (diag_width/2 - 1) * sin(radians(self.robot_theta)))
                
                globals.cd_bb_x = globals.mothership_x - (mult * (diag_width_ramp/2 - 1) * cos(radians(self.robot_theta + 90)))
                globals.cd_bb_y = globals.mothership_y - (mult * (diag_width_ramp/2 - 1) * sin(radians(self.robot_theta + 90)))
                globals.af_bb_x = globals.mothership_x - (mult * (diag_width_ramp/2 - 1) * cos(radians(self.robot_theta - 90)))
                globals.af_bb_y = globals.mothership_y - (mult * (diag_width_ramp/2 - 1) * sin(radians(self.robot_theta - 90)))

            
                rospy.loginfo("Determine Mothership Orientation State:")    
                rospy.loginfo("\tMothership: {} Position: ({},{}) Orientation: {}".format("ABC" if detected_slot.letter == 0 else "DEF", globals.mothership_x, globals.mothership_y, globals.mothership_theta))
                rospy.loginfo("\tMult: {}".format(mult))
                rospy.loginfo("\tABC Waypoint: ({},{})".format(globals.abc_x, globals.abc_y))
                rospy.loginfo("\tDEF Waypoint: ({},{})".format(globals.def_x, globals.def_y))
                rospy.loginfo("\tAF Waypoint: ({},{})".format(globals.af_x, globals.af_y))
                rospy.loginfo("\tCD Waypoint: ({},{})".format(globals.cd_x, globals.cd_y))

                commands.send_vis_command("display-mothership x:{} y:{} theta:{} abc_x:{} abc_y:{} af_x:{} af_y:{} def_x:{} def_y:{} cd_x:{} cd_y:{}".format(
                    globals.mothership_x, globals.mothership_y, globals.mothership_theta,
                    globals.abc_x, globals.abc_y,
                    globals.af_x, globals.af_y,
                    globals.def_x, globals.def_y,
                    globals.cd_x, globals.cd_y
                ))

                commands.send_drive_forward_command(-13.5)
                t.sleep(3)

                from drive_to_block_state import DriveToBlockState
                return DriveToBlockState()
            else:
                rospy.loginfo("{}H".format("A"*50))
                return self

        except Exception as e:
            print(e)
            return self
