import globals
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
        super(DetectLetterState, self).start()
        rospy.loginfo("Entering detect letter state")
	t.sleep(3)
        
    def run(self):
        try:
            if globals.tmp_slot == -1:
                detected_letter = commands.letter_srv().letter
            else:
                detected_letter = globals.tmp_slot
                globals.tmp_slot = -1
            if detected_letter == 0xFF:
                return self
            globals.current_letter = detected_letter
            commands.display_letter(detected_letter)
            commands.set_display_state(commands.LETTER)
            globals.detected_letters[globals.block_queue[0]] = detected_letter
            t.sleep(0.5)

        except Exception as e:
            print(e)
            return self

        from drive_to_mothership_state import DriveToMothershipState
        return DriveToMothershipState()
