import globals
from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t

class DetermineMothershipOrientationState(State):
    def __init__(self):
        super(DetermineMothershipOrientationState, self).__init__("Determine Mothership Orientation")

    def start(self):
        rospy.loginfo("Entering determine mothership orientation state")
        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)

    def __set_pose__(self, msg):
        self.robot_theta = msg.theta

    def run(self):
        t.sleep(5)
        try:
            detected_letter = commands.letter_srv()
            globals.current_letter = detected_letter.letter
            commands.display_letter(detected_letter.letter)
            commands.set_display_state(commands.LETTER)

            if detected_letter == 1:
                mothershipOrientation = self.robot_theta
                rospy.loginfo("MOTHERSHIP ORIENTATION:", mothershipOrientation)
            elif detected_letter == 5:
                mothershipOrientation = 180 - self.robot_theta
                rospy.loginfo("MOTHERSHIP ORIENTATION:", mothershipOrientation)
            else:
                for i in range(10):
                    rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH")


            t.sleep(1)
        except Exception as e:
            print(e)
            return self

        # from put_down_block_state import * 
        # return PutDownBlockState()
        # from find_mothership_state import *
        # return FindMothershipState(False)

    def finish(self):
        rospy.loginfo("Exiting mothership orientation state")
