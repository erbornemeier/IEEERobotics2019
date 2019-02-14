from state_machine.state import State
from std_msgs.msg import UInt8
import commands
import rospy

class NullState(State):
    def __init__(self):
        super(NullState, self).__init__("NullState")

    def start(self):
        print("Entering null state")
        rospy.wait_for_service("letter_identifier")
        letter_srv = rospy.ServiceProxy("letter_identifier", Letter)
        detected_letter = letter_srv()
        self.display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
        commands.display_letter(self.display_letter_pub, detected_letter) # Display A

    def run(self):
        print("Doing nothing")
        return self

    def finish(self):
        self.display_letter_pub.unregister()
        print("Finishing null state")
