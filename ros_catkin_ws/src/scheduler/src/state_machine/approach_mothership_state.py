import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import time as t
import math
import rospy

class ApproachMothershipState(State):
    def __init__(self, isFirstInstance):
        super(ApproachMothershipState, self).__init__("Approach Mothership State")

        self.isFirstInstance = isFirstInstance

    def start(self):
        super(ApproachMothershipState, self).start();

        self.cam_gain = 6 
        self.drive_gain = 4/27.
        self.turn_gain = 4
        self.cameraAngle = 27 
        self.rate = rospy.Rate(5)
        self.target_camera_angle = 37
        
        commands.send_cam_command(self.cameraAngle)
        commands.send_claw_command(commands.CARRY_ANGLE)
        commands.set_display_state(commands.NORMAL)

        self.found_time = t.clock()
	self.MOTHERSHIP_TIMEOUT = 0.5
	self.adjusted = False

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

    def __reset__(self):
        commands.send_cam_command(int(self.cameraAngle))
        commands.send_drive_vel_command(0, 0)

    def __camera_to_mothership__(self, mothership_pos):

        if mothership_pos.y > 0.53:
            self.cameraAngle += self.cam_gain * (mothership_pos.y - 0.5)
        elif mothership_pos.y < 0.47:
            self.cameraAngle -= self.cam_gain * (0.5 - mothership_pos.y) 

        if self.cameraAngle < 10:
            self.cameraAngle = 10
        elif self.cameraAngle > self.target_camera_angle:
            self.cameraAngle = self.target_camera_angle

        commands.send_cam_command(int(self.cameraAngle))

    def __drive_to_mothership__(self, mothership_pos):

        turn_speed = self.turn_gain * (0.5 - mothership_pos.x)
        forward_speed = self.drive_gain * (self.target_camera_angle - self.cameraAngle) + 0.3
        commands.send_drive_vel_command(forward_speed, turn_speed)
        print("Mothership angle: {}".format(mothership_pos.theta))

    def run(self):

        self.rate.sleep()
        #camera to mothership
        mothership_pos = self.__get_mothership_pos__()

        if not self.adjusted and mothership_pos.y < 0 and t.clock() - self.found_time > self.MOTHERSHIP_TIMEOUT:
            commands.send_drive_turn_command(-10)
	    self.adjusted = True
            t.sleep(2)
            return self
	elif mothership_pos.y < 0:
	    self.__reset__()
	    return self
        else:
            self.found_time = t.clock()
            self.__camera_to_mothership__(mothership_pos)

        self.__drive_to_mothership__(mothership_pos)

        #rospy.loginfo("Mothership Pos: " + str(mothership_pos.x) + ", " + str(mothership_pos.y) + " Cam Angle: " + str(self.cameraAngle))

        # TODO: Handle end condition
        if self.cameraAngle == self.target_camera_angle:
            t.sleep(0.25)
            commands.send_drive_vel_command(0, 0)

            if self.isFirstInstance:
                # Transition to determine_mothership_orientation
                from determine_mothership_orientation_state import *
                return DetermineMothershipOrientationState()
            else:
                from place_in_slot_state import *
                return PlaceInSlotState()
            
        else:
            return self

