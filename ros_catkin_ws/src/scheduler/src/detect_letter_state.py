from state_machine.state import State
from std_msgs.msg import UInt8
import commands
import rospy

class DetectLetterState(State):
    def __init__(self):
        super(DetectLetterState, self).__init__("Detect Letter")

    def start(self):
        rospy.loginfo("Entering detect letter state")

        rospy.wait_for_service("letter_identifier")
        self.letter_srv = rospy.ServiceProxy("letter_identifier", Letter)
        detected_letter = self.letter_srv()

        rospy.loginfo(detected_letter)

        self.display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
        commands.display_letter(self.display_letter_pub, detected_letter)

    def run(self):
        rospy.loginfo("Doing nothing")
        return self

    def finish(self):
        self.display_letter_pub.unregister()
        self.letter_srv.close()
        rospy.loginfo("Exiting detect letter state")
