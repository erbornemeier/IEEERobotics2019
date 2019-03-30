import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import drive_utils
import time as t
import math
import rospy



class StraightenToMothershipState(State):
    def __init__(self):
        super(StraightenToMothershipState, self).__init__("Straighten to Mothership State")

    def start(self):
        super(StraightenToMothershipState, self).start();

        self.cam_gain = 6 
        self.drive_gain = 8/27.
        self.min_speed = 0.8
        self.turn_gain = 4
        self.cameraAngle = 15 
        self.rate = rospy.Rate(5)
        self.target_camera_angle = 18.5 

        self.target_dist = 19.5
        self.approach_dist = 24 
        
        commands.send_cam_command(self.cameraAngle)
        commands.send_claw_command(commands.CARRY_ANGLE)
        commands.set_display_state(commands.NORMAL)

        self.lookup_table = [(-68, -80), (-50, -70), (-34, -60), (-24, -50),\
                        (-17.5, -40), (-11.5, -30), (-7, -20), (-2, -10),\
                        (0,3), (10, 10), (15, 20), (18.5, 30), (24, 40), (30.5, 50),\
                        (38.5, 60), (49.5, 70), (65.5, 80)]
        
        self.STRAIGHTEN_THRESH = 10 

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
        forward_speed = self.drive_gain * (self.target_camera_angle - self.cameraAngle) + self.min_speed 
        commands.send_drive_vel_command(forward_speed, turn_speed)
        #print("Mothership angle: {}".format(mothership_pos.theta))

    def __get_mothership_orientation__(self, light_angle):
        lookup_table = self.lookup_table
        for i, p in enumerate(lookup_table):
            a, orientation = p
            if light_angle < a and i != 0:
                lerp = (lookup_table[i][0] - light_angle) / \
                       (lookup_table[i][0] - lookup_table[i-1][0]) 
                return lerp*(lookup_table[i][1] - lookup_table[i-1][1]) + \
                            lookup_table[i-1][1]

    def run(self):

        last_seen = t.time()

        while self.cameraAngle != self.target_camera_angle:
            self.rate.sleep()
            #camera to mothership
            mothership_pos = self.__get_mothership_pos__()
                
            if mothership_pos.y < 0:
                if t.time() - last_seen > 2:
                    from find_mothership_state import FindMothershipState
                    return FindMothershipState()
                self.__reset__()
            else:
                last_seen = t.time()
                self.__camera_to_mothership__(mothership_pos)
                self.__drive_to_mothership__(mothership_pos)

        t.sleep(0.5)

        commands.send_drive_vel_command(0, 0)

        turnLeft = mothership_pos.theta > 0

        mothership_orientation = self.__get_mothership_orientation__(mothership_pos.theta)
        print("MOTHERSHIP ORIENTATION -> {}".format(mothership_orientation))
        #needs to correct itself
        if abs(mothership_orientation) > self.STRAIGHTEN_THRESH:

            forward_dist = (self.target_dist**2 + self.approach_dist**2 -\
                            2*self.target_dist*self.approach_dist*\
                            math.cos(math.radians(abs(mothership_orientation))) )**0.5
            turn_angle = math.asin(self.approach_dist*math.sin(\
                                            math.radians(abs(mothership_orientation)))\
                                            / forward_dist)
            turn_angle = 180 - math.degrees(turn_angle)
            if not turnLeft:
                turn_angle *= -1

            #print("FORWARD {} and TURNING {}".format(forward_dist, turn_angle))

            #t.sleep(0.5)
            drive_utils.turn(turn_angle) 
            drive_utils.drive(forward_dist)
            drive_utils.turn(90 if turn_angle < 0 else -90)

            vel = -1 if turn_angle > 0 else 1
            while True:
                self.rate.sleep()
                mothership_pos = self.__get_mothership_pos__()
                if mothership_pos.y >= 0:
                    break
                commands.send_drive_vel_command(0, vel)
        
        from approach_mothership_state import ApproachMothershipState
        return ApproachMothershipState(True) 

