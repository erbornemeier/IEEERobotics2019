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
    def __init__(self, isFirstInstance):
        super(FindMothershipState, self).__init__("Find Mothership State")
        self.isFirstInstance = isFirstInstance

    def start(self):
        super(FindMothershipState, self).start()

        commands.send_drive_vel_command(0, 1.0)
        commands.send_cam_command(15)
        commands.send_claw_command(commands.CARRY_ANGLE)
        commands.set_display_state(commands.NORMAL)

        self.rate = rospy.Rate(5)

    def __get_mothership_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.mothership_srv()
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
            commands.send_drive_vel_command(0, 0)

            from approach_mothership_state import ApproachMothershipState
            return ApproachMothershipState(self.isFirstInstance)
        else:
            return self
