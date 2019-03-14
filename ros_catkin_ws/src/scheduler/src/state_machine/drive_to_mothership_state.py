import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import time as t
import rospy

class DriveToMothershipState(State):
    def __init__(self):
        super(DriveToMothershipState, self).__init__("Drive to Mothership State")

    def start(self):
        rospy.loginfo("Entering drive to mothership state")

        self.change_display_pub = rospy.Publisher("change_display_state", UInt8, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)

        rospy.wait_for_service("drive_service")
        self.drive_srv = rospy.ServiceProxy("drive_service", Drive)

        rospy.wait_for_service("mothership")
        self.block_srv = rospy.ServiceProxy("mothership", Mothership)

        self.cam_gain = 6 
        self.drive_gain = 2/27.
        self.turn_gain = 2
        self.cameraAngle = 15
        self.rate = rospy.Rate(5)
        self.target_camera_angle = 37
        
        t.sleep(0.5)

        commands.send_cam_command(self.cam_pub, self.cameraAngle)
        commands.send_claw_command(self.claw_pub, commands.CARRY_ANGLE)
        commands.set_display_state(self.change_display_pub, commands.NORMAL)
        rospy.loginfo("Set camera to starting angle 20")

    def __get_mothership_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        # TODO: Change to motherboard service
        try:
            return self.block_srv()
        except Exception as e:
            print(e)
            mothership_pos = MothershipResponse()
            mothership_pos.x = -1
            mothership_pos.y = -1
            mothership_pos.theta = -1
            return mothership_pos

    def __reset__(self):
        #self.cameraAngle = 20
        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))
        commands.send_drive_vel_command(self.drive_srv, 0, 0)

    def __camera_to_mothership__(self, mothership_pos):

        if mothership_pos.y > 0.53:
            self.cameraAngle += self.cam_gain * (mothership_pos.y - 0.5)
        elif mothership_pos.y < 0.47:
            self.cameraAngle -= self.cam_gain * (0.5 - mothership_pos.y) 

        if self.cameraAngle < 15:
            self.cameraAngle = 15
        elif self.cameraAngle > self.target_camera_angle:
            self.cameraAngle = self.target_camera_angle

        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))

    def __drive_to_mothership__(self, mothership_pos):

        turn_speed = self.turn_gain * (0.5 - mothership_pos.x)
        forward_speed = self.drive_gain * (self.target_camera_angle - self.cameraAngle) + 0.2
        commands.send_drive_vel_command(self.drive_srv, forward_speed, turn_speed)

    def run(self):

        self.rate.sleep()

        #camera to motherboard
        mothership_pos = self.__get_mothership_pos__()
        if mothership_pos.y < 0:
            self.__reset__()
            return self
        else:
            self.__camera_to_mothership__(mothership_pos)

        self.__drive_to_mothership__(mothership_pos)

        #rospy.loginfo("Mothership Pos: " + str(mothership_pos.x) + ", " + str(mothership_pos.y) + " Cam Angle: " + str(self.cameraAngle))

        if self.cameraAngle == self.target_camera_angle:
            commands.send_drive_vel_command(self.drive_srv, 0, 0)
            from place_in_slot_state import *
            return PlaceInSlotState()
        else:
            return self
        
    def finish(self):
        self.drive_srv.close()
        self.cam_pub.unregister()
        self.change_display_pub.unregister()
        self.block_srv.close()
        rospy.loginfo("Exiting drive to block state")

