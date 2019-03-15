#!/usr/bin/python
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import rospy

PICKUP_ANGLE = 40
DROP_ANGLE = 10
CARRY_ANGLE = 13

CLAW_OPEN = False
CLAW_CLOSED = True


# Publishers
print('publish start')
claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
display_letter_pub = rospy.Publisher("change_display_state", UInt8, queue_size=1)
grip_pub = rospy.Publisher("grip_command", Bool, queue_size=1)
display_block_pub = rospy.Publisher("display_block", UInt8, queue_size=1)
display_pub = rospy.Publisher("change_display_state", UInt8, queue_size=1)
drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
print('publish end')

#Services
print("service")
rospy.wait_for_service("block_pos")
block_srv = rospy.ServiceProxy("block_pos", Block)
rospy.wait_for_service("mothership")
mothership_srv = rospy.ServiceProxy("mothership", Mothership)
rospy.wait_for_service("letter_identifier")
letter_srv = rospy.ServiceProxy("letter_identifier", Letter)
print('service end')

def send_claw_command(cmd):
    msg = UInt8()
    msg.data = cmd
    claw_pub.publish(msg)

def send_grip_command(is_closed):
    msg = Bool()
    msg.data = is_closed
    grip_pub.publish(msg)

def send_cam_command(angle):
    msg = UInt8()
    msg.data = angle
    cam_pub.publish(msg)

def send_drive_vel_command(x, theta):
    msg = Pose2D()
    msg.x = x
    msg.y = 0
    msg.theta = theta
    drive_pub.publish(msg)

def send_drive_forward_command(x):
    msg = Pose2D()
    msg.x = x
    msg.y = 1
    msg.theta = 0
    drive_pub.publish(msg)

def send_drive_turn_command(theta):
    msg = Pose2D()
    msg.x = 0
    msg.y = 2
    msg.theta = theta
    drive_pub.publish(msg)

def display_letter(letter):
    msg = UInt8()
    msg.data = letter
    display_letter_pub.publish(msg)

def display_block_command(x, y):
    msg = UInt8()
    msg.data = ((x&0xFF) << 4) | (y&0xFF)
    display_block_pub.publish(msg)

NORMAL = 0
LETTER = 1
WAITING = 2
def set_display_state(state):
    msg = UInt8()
    msg.data = state
    display_pub.publish(msg)
