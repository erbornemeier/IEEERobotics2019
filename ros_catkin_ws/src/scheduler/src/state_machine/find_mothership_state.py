import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import drive_utils
import geometry_utils
import time as t
import rospy
from math import *

class FindMothershipState(State):
    def __init__(self, isFirstInstance):
        super(FindMothershipState, self).__init__("Find Mothership State")
        self.isFirstInstance = isFirstInstance

    def start(self):
        super(FindMothershipState, self).start()

        commands.send_drive_vel_command(0, 0.6)
        commands.send_cam_command(15)
        commands.send_claw_command(commands.CARRY_ANGLE)
        commands.set_display_state(commands.NORMAL)

        self.rate = rospy.Rate(5)
        self.max_size_contour = -1
        self.angle_of_max = 0

        self.start_time = t.time()
        self.timeout = 27
        self.drive_dst = 40

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

    def __get_potential_mothership_pos__(self):
        try:
            return commands.big_orange_srv()
        except Exception as e:
            mothership_pos = MothershipResponse()
            mothership_pos.x = -1
            mothership_pos.y = -1
            mothership_pos.theta = -1
            return mothership_pos

    def run(self):
        self.rate.sleep()
        mothership_pos = self.__get_mothership_pos__()

        if t.time() - self.start_time > self.timeout:
            commands.send_drive_vel_command(0,0)

            target_angle = self.angle_of_max - 60
            pt_x = 48 + self.drive_dst * cos(radians(target_angle))
            pt_y = 48 + self.drive_dst * sin(radians(target_angle))

            tmp_bad_points = []
            for p in drive_utils.grid:
                if geometry_utils.pointInEllipse(48, 48, self.angle_of_max, p[0], p[1], 100, 30) \
                    and not geometry_utils.pointInEllipse(48, 48, self.angle_of_max, p[0], p[1], 12, 30):
                    tmp_bad_points.append(p)
                    globals.bad_points.add(p)

            drive_utils.go_to_point((pt_x, pt_y))

            for p in tmp_bad_points:
                globals.bad_points.remove(p)
            return FindMothershipState('if you are reading this help me')


        if mothership_pos.y >= 0:
            self.mothership_found = True
            commands.send_drive_vel_command(0,0)

            from straighten_to_mothership_state import StraightenToMothershipState
            return StraightenToMothershipState()
        else:
            drive_utils.wait_for_pose_update()
            pot_mothership_pos = self.__get_potential_mothership_pos__()
            if pot_mothership_pos.x > self.max_size_contour:
                self.max_size_contour = pot_mothership_pos.x
                self.angle_of_max = drive_utils.robot_theta 
                print(self.angle_of_max)
            return self
