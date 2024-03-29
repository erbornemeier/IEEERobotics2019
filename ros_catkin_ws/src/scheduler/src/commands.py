#!/usr/bin/python
from std_msgs.msg import UInt8, Bool, String
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import rospy
import drive_utils

PICKUP_ANGLE = 40
DROP_ANGLE = 13
CARRY_ANGLE = 16
CLAW_DETERMINE_MOTHERSHIP_ANGLE = 58 

CAMERA_DETERMINE_MOTHERSHIP_ANGLE = 45

CLAW_OPEN = False
CLAW_CLOSED = True

# Publishers
claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size=1)
grip_pub = rospy.Publisher("grip_command", Bool, queue_size=1)
display_block_pub = rospy.Publisher("display_block", UInt8, queue_size=1)
display_pub = rospy.Publisher("change_display_state", UInt8, queue_size=3)
drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=3)
cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=3)
vis_cmd_pub = rospy.Publisher("vis_command", String, queue_size=1)
print("Publishers Initialized")

#Services
rospy.wait_for_service("block_pos")
block_srv = rospy.ServiceProxy("block_pos", Block)
rospy.wait_for_service("block_pos_close")
block_srv_close = rospy.ServiceProxy("block_pos_close", Block)
rospy.wait_for_service("mothership")
mothership_srv = rospy.ServiceProxy("mothership", Mothership)
rospy.wait_for_service("letter_identifier")
letter_srv = rospy.ServiceProxy("letter_identifier", Letter)
rospy.wait_for_service("big_orange")
big_orange_srv = rospy.ServiceProxy("big_orange", Mothership)
rospy.wait_for_service("slot_identifier")
slot_srv = rospy.ServiceProxy("slot_identifier", Letter) 
print("Services Initialized")

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
    if x==0 and theta == 0:
        drive_utils.stop()
    else:
        msg = Pose2D()
        msg.x = x
        msg.y = 0
        msg.theta = theta
        drive_pub.publish(msg)

def send_drive_forward_command(x):
    msg = Pose2D()
    msg.x = x
    msg.y = 1
    drive_pub.publish(msg)
    received = drive_utils.wait_for_msg_received()
    if not received:
        send_drive_forward_command(x)

def send_drive_turn_command(theta):
    msg = Pose2D()
    msg.y = 2
    msg.theta = theta
    drive_pub.publish(msg)
    received = drive_utils.wait_for_msg_received()
    if not received:
        send_drive_turn_command(theta)

def send_override_position_command(new_x, new_y):
    msg = Pose2D()
    msg.x = new_x
    msg.theta = new_y
    msg.y = 3 #override pos
    drive_pub.publish(msg)

def send_drop_block_command(distance, angle):
    msg = Pose2D()
    msg.x = distance
    msg.y = 4
    msg.theta = angle
    drive_pub.publish(msg)
    received = drive_utils.wait_for_msg_received()
    if not received:
        send_drop_block_command(distance, angle)
    
def send_pickup_command():
    msg = Pose2D()
    msg.y = 5
    drive_pub.publish(msg)
    received = drive_utils.wait_for_msg_received()
    if not received:
        send_pickup_command()

def send_look_in_mothership_command(distance):
    msg = Pose2D()
    msg.x = distance
    msg.y = 6
    drive_pub.publish(msg)
    received = drive_utils.wait_for_msg_received()
    if not received:
        send_look_in_mothership_command(distance)

def display_letter(letter):
    msg = UInt8()
    msg.data = letter
    display_letter_pub.publish(msg)

def display_block_command(x, y):
    msg = UInt8()
    msg.data = ((x&0xFF) << 4) | (y&0xF)
    display_block_pub.publish(msg)

def send_vis_command(data):
    msg = String()
    msg.data = data
    vis_cmd_pub.publish(msg)
NORMAL = 0
LETTER = 1
WAITING = 2
LOADING = 3
FINISHED = 4
def set_display_state(state):
    msg = UInt8()
    msg.data = state
    display_pub.publish(msg)
