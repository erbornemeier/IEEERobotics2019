import globals
from pypaths import astar
#import astar #custom astar
import commands
from geometry_utils import *
import time as t
import math
import rospy
from geometry_msgs.msg import Pose2D

# POSE INFORMATION
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

def wait_for_pose_update():
    global robot_x
    robot_x = -1
    while robot_x == -1:
        print('waiting for pose update')
        t.sleep(0.25)

def wait_for_pose_change():
    #wait_for_pose_update()
    global robot_x, robot_y, robot_theta
    pose_before = (robot_x, robot_y, robot_theta)
    while (robot_x, robot_y, robot_theta) == pose_before:
        print('waiting for pose change')
        t.sleep(0.25)

# PATH FINDING
RESOLUTION = 4
MARGIN = 6
FIRST_POINT_IN = int(math.ceil(MARGIN/float(RESOLUTION))*RESOLUTION)
print(FIRST_POINT_IN)

def __custom_neighbors__( height, width ):
    def func( coord ):
        neighbor_list = [(coord[0]+i,coord[1]+j) 
                            for i in range(-RESOLUTION, RESOLUTION+1, RESOLUTION)
                            for j in range(-RESOLUTION, RESOLUTION+1, RESOLUTION) ]
        return [ c for c in neighbor_list
                 if c != coord
                 and c not in globals.bad_points
                 and c[0] >= MARGIN and c[0] <= width-MARGIN
                 and c[1] >= MARGIN and c[1] <= height-MARGIN ]
    return func

finder = astar.pathfinder(neighbors=__custom_neighbors__(12*8, 12*8),
                    distance=astar.absolute_distance,
                    cost=astar.fixed_cost(1))
def __optimize_path__(path):
    #optimized_path = [path[0]]
    optimized_path = []
    for i, p in enumerate(path[1:-1]):
        if p != avg(path[i], path[i+2]):
            optimized_path.append(p)
            pass
    #optimized_path.append(path[-1])
    return optimized_path

grid = [(i,j) for i in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)\
              for j in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]
def __approx_to_grid__(pt):
    closest = sorted(grid, key=lambda x: dist(x, pt))
    for p in closest:
        if p not in globals.bad_points:
            return p
    return closest[0]

def __get_path__(from_pt, to_pt):
    #from_pt = tuple([int(round(a/RESOLUTION)*RESOLUTION) for a in from_pt])
    #to_pt_approx = tuple([int(round(a/RESOLUTION)*RESOLUTION) for a in to_pt])
    from_pt_approx = __approx_to_grid__(from_pt)
    to_pt_approx = __approx_to_grid__(to_pt)
    print("FROM PT APRPOX: {}".format(from_pt_approx))
    print("TO PT APRPOX: {}".format(to_pt_approx))

    path = finder(from_pt_approx, to_pt_approx)[1]
    print("PREOPTIMIZED PATH: {}".format(path))
    opt_path = __optimize_path__(path) 
    if MARGIN <= to_pt[0] <= 12*8-MARGIN and MARGIN <= to_pt[1] <= 12*8-MARGIN:
        opt_path.append(to_pt)
    else:
        opt_path.append(to_pt_approx)
    return opt_path

def get_drive_instructions(to_pt):
    global robot_x, robot_y, robot_theta
    wait_for_pose_update()
    robot_pos = (robot_x, robot_y)
    dx, dy = to_pt[0]-robot_pos[0], to_pt[1]-robot_pos[1]
    forward_dist = max(0, dist(robot_pos, to_pt)) 
    turn_angle = math.degrees(math.atan2(dy, dx)) - robot_theta 
    turn_angle = bound_angle(turn_angle)
    return turn_angle, forward_dist

def drive(forward_dist):
    print("DRIVING: {}".format(forward_dist))
    if abs(forward_dist) > 0.1:
        commands.send_drive_forward_command(forward_dist)
        wait_for_pose_change()

def turn(turn_angle):
    print("TURNING: {}".format(turn_angle))
    if abs(turn_angle) > 0.1:
        commands.send_drive_turn_command(turn_angle)
        wait_for_pose_change()

def drive_to_point(to_pt):
    turn_angle, forward_dist = get_drive_instructions(to_pt)
    print("TURNING: {} THEN DRIVING {}".format(turn_angle, forward_dist))
    turn(turn_angle)
    drive(forward_dist) 

def get_approach_point(from_pt, to_pt, approach_dist):
    approach_perc = approach_dist / dist(from_pt, to_pt)
    if approach_perc > 1:
        return to_pt
    return avg(from_pt, to_pt, weight=approach_perc)

def go_to_point(to_pt, approach_dist=0):
    global robot_x, robot_y, robot_theta
    wait_for_pose_update()
    robot_pos = (robot_x, robot_y)
    if approach_dist != 0:
        to_pt = get_approach_point(robot_pos, to_pt, approach_dist)
    path = __get_path__(robot_pos, to_pt)
    print("PATH: {}".format(path))
    #TODO change to instructions to send to arduino
    for p in path:
        drive_to_point(p)
    
'''
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


def drive_safely(target_pos, redo=False):
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
        if not redo:
            print ("Target in mothership bounds")
            dx, dy = target_pos[0]-globals.mothership_x, target_pos[1]-globals.mothership_y
            to_target_angle = math.atan2(dy, dx)  
            int_point = (globals.mothership_x + 28*math.cos(to_target_angle),\
                         globals.mothership_y + 28*math.sin(to_target_angle))
            #go around to proper point
            approach_point = drive_safely(int_point, True)
            go_to_point(approach_point)
            go_to_point(int_point)
            print("Done pathfinding, going to target")
        #no path found, robot is in mothership bounds
        else: 
            closest_pt = None
            closest_dist = float('inf')
            for p in mothership_pts:
                dist = __dist__(p, current_pos_xy)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_pt = p
            go_to_point(closest_pt)

    return None
'''
