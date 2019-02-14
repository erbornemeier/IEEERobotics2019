from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t

class DetectLetterState(State):
    def __init__(self):
        super(DetectLetterState, self).__init__("Detect Letter")

    def start(self):
        rospy.loginfo("Entering detect letter state")

        self.display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
        rospy.wait_for_service("letter_identifier")
        self.letter_srv = rospy.ServiceProxy("letter_identifier", Letter)
        
    def run(self):
        t.sleep(5)
        try:
            detected_letter = self.letter_srv()
        except Exception as e:
            print(e)
            return self
        commands.display_letter(self.display_letter_pub, detected_letter.letter)
        rospy.loginfo(detected_letter.letter)

        from put_down_block_state import * 
        return PutDownBlockState()

    def finish(self):
        self.display_letter_pub.unregister()
        self.letter_srv.close()
        rospy.loginfo("Exiting detect letter state")
