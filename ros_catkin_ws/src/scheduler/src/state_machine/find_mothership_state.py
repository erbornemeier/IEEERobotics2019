import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import time as t
import rospy

class FindMothershipState(State):
    def __init__(self):
        super(FindMothershipState, self).__init__("Find Mothership State")

    def start(self):
        rospy.loginfo("Entering find mothership state")

        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.change_display_pub = rospy.Publisher("change_display_state", UInt8, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)

        rospy.wait_for_service("mothership")
        self.block_srv = rospy.ServiceProxy("mothership", Mothership)

        t.sleep(0.5)

        commands.send_drive_vel_command(self.drive_pub, 0, 1.0)
        commands.send_cam_command(self.cam_pub, 15)
        commands.send_claw_command(self.claw_pub, commands.CARRY_ANGLE)
        commands.set_display_state(self.change_display_pub, commands.NORMAL)

        self.rate = rospy.Rate(5)

    def __get_mothership_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return self.block_srv()
        except Exception as e:
            print(e)
            mothership_pos = MothershipResponse()
            mothership_pos.x = -1
            mothership_pos.y = -1
            mothership_pos.theta = -1
            return mothership_pos

    def run(self):

        self.rate.sleep()

        mothership_pos = self.__get_mothership_pos__()

        if mothership_pos.y >= 0:
            self.mothership_found = True
            commands.send_drive_vel_command(self.drive_pub, 0, 0)

            from drive_to_mothership_state import *
            return DriveToMothershipState()
        else:
            return self
        
    def finish(self):
        self.drive_pub.unregister()
        self.cam_pub.unregister()
        self.change_display_pub.unregister()
        self.block_srv.close()
        rospy.loginfo("Exiting find mothership state")

