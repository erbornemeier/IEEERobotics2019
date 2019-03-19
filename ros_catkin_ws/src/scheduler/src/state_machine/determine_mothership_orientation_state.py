import globals
from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t
from geometry_msgs.msg import Pose2D

class DetermineMothershipOrientationState(State):
    def __init__(self):
        super(DetermineMothershipOrientationState, self).__init__("Determine Mothership Orientation")

    def start(self):
        rospy.loginfo("Entering determine mothership orientation state")
        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)
        commands.send_cam_command(commands.CAMERA_DETERMINE_MOTHERSHIP_ANGLE)
        t.sleep(1)
        commands.send_claw_command(commands.CLAW_DETERMINE_MOTHERSHIP_ANGLE)
        t.sleep(1)
        commands.send_drive_forward_command(6)
        t.sleep(2)

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def run(self):
        t.sleep(5)
        try:
            detected_slot = commands.slot_srv()
            globals.current_letter = detected_slot.data
            commands.display_letter(1 if detected_slot.data == 0 else 5)
            commands.set_display_state(commands.LETTER)

            if detected_slot == 1 or detected_slot == 5:
                # TODO: What if robot is turned?
                globals.mothership_theta = self.robot_theta if detected_slot == 0 else 180 - self.robot_theta
                globals.mothership_x = self.robot_x
                globals.mothership_y = self.robot_y
                rospy.loginfo("MOTHERSHIP ORIENTATION: ", globals.mothership_theta, "POSITION:", self.mothership_x, " ", self.mothership_y)
            else:
                for i in range(10):
                    rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH")

            commands.send_drive_forward_command(-10)

            rospy.Rate(0.2).sleep()

            from find_mothership_state import *
            return FindMothershipState(True)
        except Exception as e:
            print(e)
            return self

        # from put_down_block_state import * 
        # return PutDownBlockState()
        # from find_mothership_state import *
        # return FindMothershipState(False)

    def finish(self):
        rospy.loginfo("Exiting mothership orientation state")
