from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D

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

def display_letter(display_letter_pub, letter):
    msg = UInt8()
    msg.data = letter
    print("Sending: " + str(msg.data))
    display_letter_pub.publish(msg)

def send_drive_command(drive_pub, x, y, theta):
    msg = Pose2D()
    msg.x = x
    msg.y = y
    msg.theta = theta
    drive_pub.publish(msg)
