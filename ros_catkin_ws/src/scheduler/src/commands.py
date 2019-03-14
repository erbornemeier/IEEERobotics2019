from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D
from scheduler.srv import *
import rospy

PICKUP_ANGLE = 40
DROP_ANGLE = 10
CARRY_ANGLE = 13

CLAW_OPEN = False
CLAW_CLOSED = True

def send_claw_command(claw_pub, cmd):
    msg = UInt8()
    msg.data = cmd
    claw_pub.publish(msg)

def send_grip_command(grip_pub, is_closed):
    msg = Bool()
    msg.data = is_closed
    grip_pub.publish(msg)

def send_cam_command(cam_pub, angle):
    msg = UInt8()
    msg.data = angle
    cam_pub.publish(msg)

def send_drive_vel_command(drive_srv, x, theta):
    req = DriveRequest() 
    req.type = 0
    req.forward = x
    req.theta = theta
    while True:
        response = drive_srv(req)
        if response is not None:
            break
        else:
            t.sleep(1)

def send_drive_forward_command(drive_srv, x):
    req = DriveRequest() 
    req.type = 1
    req.forward = x
    req.theta = 0 
    while True:
        response = drive_srv(req)
        if response is not None:
            break
        else:
            t.sleep(1)

def send_drive_turn_command(drive_srv, theta):
    req = DriveRequest() 
    req.type = 1
    req.forward = 0
    req.theta = theta
    while True:
        response = drive_srv(req)
        if response is not None:
            break
        else:
            t.sleep(1)

def display_letter(display_letter_pub, letter):
    msg = UInt8()
    msg.data = letter
    display_letter_pub.publish(msg)

def display_block_command(led_pub, x, y):
    msg = UInt8()
    msg.data = x & 0xFF
    msg.data <<= 4
    msg.data |= y & 0xF
    led_pub.publish(msg)

NORMAL = 0
LETTER = 1
WAITING = 2
def set_display_state(display_pub, state):
    msg = UInt8()
    msg.data = state
    display_pub.publish(msg)

