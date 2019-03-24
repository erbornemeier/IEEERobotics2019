import globals
import commands
import time as t
import math
import rospy
from geometry_msgs.msg import Pose2D

x, y = 0, 1
robot_x = -1
robot_y = -1
robot_theta = -1
def __set_pose__(msg):
    global robot_x, robot_y, robot_theta
    robot_x = msg.x
    robot_y = msg.y
    robot_theta = msg.theta

pose_sub = rospy.Subscriber('robot_pose', Pose2D, __set_pose__)

def __ccw__(a, b, c):
    return (c[y]-a[y])*(b[x]-a[x]) > (b[y]-a[y])*(c[x]-a[x])

def __intersection__(l1, l2):
    a, b = l1
    c, d = l2
    return __ccw__(a,c,d) != __ccw__(b,c,d) and __ccw__(a,b,c) != __ccw__(a,b,d)

def __collides__(segment, box):

    #connect box points as a series of line segments
    box_lines = zip(box, box[1:] + [box[0]])
    box_lines.append((box[0], box[2]))
    box_lines.append((box[1], box[3]))

    #check if line segment collides with any box segments
    for box_line in box_lines:
        print("Trying {} -> {}".format(segment[0], segment[1]))
        if __intersection__(segment, box_line):
            print("Collided with {}".format(box_line))
            return True
    return False

def __dist__(p1, p2):
    return sum((z[1] - z[0])**2 for z in zip(p1, p2))**0.5

def go_to_point(target_pos, approach_dist=0):
    global robot_x, robot_y, robot_theta 
    robot_x = -1
    while (robot_x == -1):
        print('waiting for pose')
        t.sleep(0.5)
        pass
    dx, dy = target_pos[0]-robot_x, target_pos[1]-robot_y
    forward_dist = (dx**2 + dy**2)**0.5 - approach_dist 
    forward_dist = max(0, forward_dist)
    turn_angle = (math.atan2(dy, dx) * 180.0/3.14159) - robot_theta
    if turn_angle > 180:
        turn_angle -= 360
    if turn_angle < -180:
        turn_angle += 360
    print("TURNING: {} THEN DRIVING {}".format(turn_angle, forward_dist))
    pose_before = (robot_x, robot_y, robot_theta)
    if abs(turn_angle) > 0.1:
        commands.send_drive_turn_command(turn_angle)
        while (robot_x, robot_y, robot_theta) == pose_before:
            print('waiting for change')
            t.sleep(0.5)
            pass
    pose_before = (robot_x, robot_y, robot_theta)
    if forward_dist > 0.1:
        commands.send_drive_forward_command(forward_dist)
        while (robot_x, robot_y, robot_theta) == pose_before:
            print('waiting for change')
            t.sleep(0.5)
            pass


def drive_safely(target_pos, approach_dist=0):
    global robot_x, robot_y, robot_theta 
    robot_x = -1
    while (robot_x == -1):
        print('waiting for pose')
        t.sleep(0.5)
        pass
    current_pos_xy = (robot_x, robot_y) 
    current_pos = (robot_x, robot_y, robot_theta)

    #get line segment to target
    current_to_target = (current_pos_xy, target_pos)
    #form box around mothership
    mothership_box = [(globals.abc_bb_x, globals.abc_bb_y),
                      (globals.af_bb_x, globals.af_bb_y),
                      (globals.def_bb_x, globals.def_bb_y),
                      (globals.cd_bb_x, globals.cd_bb_y)]

    mothership_pts = [(globals.abc_x, globals.abc_y),
                      (globals.af_x, globals.af_y),
                      (globals.def_x, globals.def_y),
                      (globals.cd_x, globals.cd_y)]

    print(mothership_box)

    #if a collision of mothership is predicted
    if (__collides__(current_to_target, mothership_box)):
        print("FINDING PATH TO TARGET")
        #find path to target
        for b in mothership_pts:
            if not (__collides__( (current_pos_xy, b), mothership_box) or 
                    __collides__( (b, target_pos)    , mothership_box)):
                return b
        #no path found, target is in mothership bounds
        print ("Target in mothership bounds")
        dx, dy = target_pos[0]-globals.mothership_x, target_pos[1]-globals.mothership_y
        to_target_angle = math.atan2(dy, dx)  
        int_point = (globals.mothership_x + 28*math.cos(to_target_angle),\
                     globals.mothership_y + 28*math.sin(to_target_angle))
        #go around to proper point
        approach_point = drive_safely(int_point)
        go_to_point(approach_point)
        go_to_point(int_point)
        print("Done pathfinding, going to target")

    return None
