from __future__ import print_function
import globals
from pypaths import astar
import commands
from geometry_utils import *
import time as t
import math
import sys
import rospy
from geometry_msgs.msg import Pose2D
import RPi.GPIO as RPIO
from skimage.measure import label
import numpy as np

# setup ack pin
ack_pin = 40
stop_pin = 38
start_pin = 36
RPIO.setmode(RPIO.BOARD)
RPIO.setup(ack_pin, RPIO.IN, pull_up_down=RPIO.PUD_UP)
RPIO.setup(stop_pin, RPIO.OUT)
RPIO.output(stop_pin, RPIO.LOW)
RPIO.setup(start_pin, RPIO.IN, pull_up_down=RPIO.PUD_DOWN)

# PATH FINDING
RESOLUTION = 3
MARGIN = 8
FIRST_POINT_IN = int(math.ceil(MARGIN/float(RESOLUTION))*RESOLUTION)
grid = [(i,j) for i in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)\
              for j in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]

def set_grid(resolution, margin):
    global RESOLUTION, MARGIN, FIRST_POINT_IN, grid

    RESOLUTION = resolution
    MARGIN = margin
    FIRST_POINT_IN = int(math.ceil(MARGIN/float(RESOLUTION))*RESOLUTION)
    grid = [(i,j) for i in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)\
              for j in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]
    commands.send_vis_command("init-pathfinding resolution:{} margin:{}".format(RESOLUTION, MARGIN))

# POSE INFORMATION
x, y = 0, 1
robot_x = -1
robot_y = -1
robot_theta = -1
TIMEOUT = 2
def __set_pose__(msg):
    global robot_x, robot_y, robot_theta
    robot_x = msg.x
    robot_y = msg.y
    robot_theta = msg.theta
pose_sub = rospy.Subscriber('robot_pose', Pose2D, __set_pose__)

def wait_for_pose_update():
    start_time = t.time()
    global robot_x
    robot_x = -1
    #dots = 0
    #print("")
    while robot_x == -1 and t.time() - start_time < TIMEOUT:
        #print('\r                             ', end="")
        #print('\rwaiting for pose update{}'.format('.'*dots), end="")
        t.sleep(0.05)
        #dots = (dots + 1) % 4
    #print("")

def wait_for_msg_received():
    start_time = t.time()
    while RPIO.input(ack_pin) == RPIO.LOW and t.time() - start_time < TIMEOUT:
        #print("Waiting for busy pin to be set")
        t.sleep(0.05)
    if t.time() - start_time >= TIMEOUT:
        return False
    return True

def wait_for_start_button():
    while RPIO.input(start_pin) == RPIO.LOW:
        pass

def wait_for_pose_change():
    #dots = 0
    while RPIO.input(ack_pin) == RPIO.HIGH:
        #print('\r                             ', end="")
        #print('\rwaiting for pose change{}'.format('.'*dots), end="")
        t.sleep(0.05)
        #dots = (dots + 1) % 4
    wait_for_pose_update()

traversed_astar_points = set()
def __custom_neighbors__( height, width ):
    def func( coord ):
        #traversed_astar_points.add(coord[:2])
        neighbor_list = [(coord[0], coord[1], coord[2]+angle) \
                        for angle in range(-135, 181, 45)]
        next_x = coord[0] + math.cos(math.radians(coord[2]))*\
                            (1 if coord[2]%90==0 else 2**0.5)*RESOLUTION
        next_y = coord[1] + math.sin(math.radians(coord[2]))*\
                            (1 if coord[2]%90==0 else 2**0.5)*RESOLUTION

        neighbor_list.append((int(round(next_x)), int(round(next_y)), coord[2]))

        neighbor_list = [ c for c in neighbor_list
                 if c != coord
                 and c[:2] not in globals.bad_points
                 and c[:2] not in traversed_astar_points
                 and c[0] >= MARGIN and c[0] <= width-MARGIN
                 and c[1] >= MARGIN and c[1] <= height-MARGIN ]


        return neighbor_list

    return func

def __custom_cost__(a, b):
    if a[2] != b[2]:
        return 8 # 5 and 10 was decent
    elif a[2] % 90 == 0:
        return 1
    else:
        return 2**0.5

finder = astar.pathfinder(neighbors=__custom_neighbors__(12*8, 12*8),
                    distance=astar.absolute_distance,
                    cost=__custom_cost__)
def __optimize_path__(path):
    if dist(path[0], (robot_x, robot_y)) > RESOLUTION:
        optimized_path = [path[0]]
    else:
        optimized_path = []
    for i, p in enumerate(path[1:-1]):
        if path[i][:2] == p[:2]:
            continue
        
        if p[:2] != avg(path[i], path[i+2]):
            optimized_path.append(p)
    #optimized_path.append(path[-1])
    return optimized_path

def __approx_to_grid__(pt):
    closest = sorted(grid, key=lambda x: dist(x, pt[:2]))
    snap_angle = int(round(pt[2]/float(45)))*45
    for p in closest:
        if p not in globals.bad_points:
            return (p[0], p[1], snap_angle)
    print("COULD NOT APPROXIMATE POINT TO A VALID POINT, CHOSE CLOSEST")
    return tuple([closest[0]] + [snap_angle])

regions = dict()
grid_changed = True
def __assign_regions__():
    global regions
    good_pts = [[1 for x in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]\
                   for y in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]
    for y in range(len(good_pts)):
        for x in range(len(good_pts[0])):
            if (x*RESOLUTION+FIRST_POINT_IN, 12*8 - (y*RESOLUTION+FIRST_POINT_IN))\
                 in globals.bad_points:
                good_pts[y][x] = 0

    labels = label(np.array(good_pts), connectivity=2)
    for y in range(len(labels)):
        for x in range(len(labels[0])):
            regions[(x*RESOLUTION+FIRST_POINT_IN, 12*8 - (y*RESOLUTION+FIRST_POINT_IN))]\
                = labels[y][x]
    print (labels)
    global grid_changed
    grid_changed = False

def __path_exists__(from_pt, to_pt):
    if grid_changed:
        __assign_regions__()
    return regions[from_pt[:2]] == regions[to_pt[:2]]

def __get_path__(from_pt, to_pt):
      
    from_pt_approx = __approx_to_grid__(from_pt)
    to_pt_approx = __approx_to_grid__(to_pt)
    
    if __path_exists__(from_pt_approx, to_pt_approx):
        path = finder(from_pt_approx, to_pt_approx)[1]
        print("PREOPTIMIZED PATH: {}".format(path))

        opt_path = __optimize_path__(path) 
        print("OPTIMIZED PATH: {}".format(opt_path))
        if len(opt_path) == 0:
            opt_path.append(to_pt)
            return opt_path

        #final point in margin
        if MARGIN <= to_pt[0] <= 12*8-MARGIN and MARGIN <= to_pt[1] <= 12*8-MARGIN and \
          dist(opt_path[-1], to_pt) < RESOLUTION*2:
            opt_path = opt_path[:-1] 
            opt_path.append(to_pt)
            print('Disregarding final approximation point, super close to point')

        elif MARGIN <= to_pt[0] <= 12*8-MARGIN and MARGIN <= to_pt[1] <= 12*8-MARGIN and \
          dist(opt_path[-1], to_pt) >= RESOLUTION*2:
            opt_path.append(to_pt)

        #final point outside of margin 
        else:
            #opt_path.append(to_pt_approx)
            new_pt_approx = [to_pt[0], to_pt[1]]
            if to_pt[0] < MARGIN:
                new_pt_approx[0] = MARGIN
            elif to_pt[0] > (12*8 - MARGIN):
                new_pt_approx[0] = (12*8 - MARGIN)
            if to_pt[1] < MARGIN:
                new_pt_approx[1] = MARGIN
            elif to_pt[1] > (12*8 - MARGIN):
                new_pt_approx[1] = (12*8 - MARGIN)
            opt_path.append(tuple(new_pt_approx))
            print("Final point outside of margin:")
            print("\tOriginal Point: {}".format(to_pt))
            print("\tUpdated Point: {}".format(new_pt_approx))

        return opt_path

    else:
        print('No path -- doing something else')
        return False


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
    #print("DRIVING: {}".format(forward_dist))
    global robot_x, robot_y, robot_theta
    wait_for_pose_update()
   
    if forward_dist < 0:
        if robot_x >= 12*8 - MARGIN or robot_x <= MARGIN\
            or robot_y >= 12*8 - MARGIN or robot_y <= MARGIN:
            return False

    if abs(forward_dist) > 0.1:
        commands.send_drive_forward_command(forward_dist)
        wait_for_pose_change()
    return True

def turn(turn_angle):
    #print("TURNING: {}".format(turn_angle))
    if abs(turn_angle) > 0.1:
        commands.send_drive_turn_command(turn_angle)
        wait_for_pose_change()

def stop():
    RPIO.output(stop_pin, RPIO.HIGH)
    t.sleep(0.25)
    RPIO.output(stop_pin, RPIO.LOW)

def drive_to_point(to_pt):
    turn_angle, forward_dist = get_drive_instructions(to_pt)
    #print("TURNING: {} THEN DRIVING {}".format(turn_angle, forward_dist))
    turn(turn_angle)
    drive(forward_dist) 

def get_approach_point(from_pt, to_pt, approach_dist):

    closest = sorted(grid, key=lambda x: dist(x, from_pt[:2]))

    from_pt_approx = __approx_to_grid__(from_pt)
    for p in closest:
        if p not in globals.bad_points:
            if approach_dist*0.95 <= dist(p, to_pt) <= approach_dist*1.25 \
             and __path_exists__(p, from_pt_approx):
                mid_pt = list(avg(p, to_pt, weight=0.5)) + [0]
                mid_pt = __approx_to_grid__(mid_pt)
                if mid_pt not in globals.bad_points:
                    return p
    return False

def go_to_point(to_pt, approach_dist=0):
    global robot_x, robot_y, robot_theta
    wait_for_pose_update()
    robot_pos = (robot_x, robot_y, robot_theta)
    robot_grid = __approx_to_grid__(robot_pos)

    #get approach point at least half approach_dist away from target
    if approach_dist != 0:
        to_pt = get_approach_point(robot_pos, to_pt, approach_dist)
        if to_pt is False:
            return False
        print("APPROXIMATED APPROACH POINT {}".format(to_pt))

    to_pt = (to_pt[0], to_pt[1], 0)
    path = __get_path__(robot_pos, to_pt)
    if path is False:
        return False

    commands.send_vis_command("draw-path {}".format([robot_pos] + path)) 

    #if approach_dist != 0:
    #path = path[:-1]

    print("PATH: {}".format(path))
    for p in path:
        drive_to_point(p)
    return True

def remove_edge_points():
    for p in grid:
        if p[0] == FIRST_POINT_IN or p[0] == 12*8 - FIRST_POINT_IN\
            or p[1] == FIRST_POINT_IN or p[1] == 12*8 - FIRST_POINT_IN:
                if p in globals.bad_points:
                    globals.bad_points.remove(p)
                if p in globals.mothership_bad_points:
                    globals.mothership_bad_points.remove(p)
    
    globals.bad_points.add((FIRST_POINT_IN, FIRST_POINT_IN))
    globals.bad_points.add((FIRST_POINT_IN, 12*8 - FIRST_POINT_IN))
    globals.bad_points.add((12*8 - FIRST_POINT_IN, FIRST_POINT_IN))
    globals.bad_points.add((12*8 - FIRST_POINT_IN, 12*8 - FIRST_POINT_IN))

    global grid_changed
    grid_changed = True
        

def add_bad_points_around_block(x, y):
    '''
    x,y is the foot location, for example 1, 1 is in cell 1, 1
    '''
    x = (x * 12) + 6
    y = (y * 12) + 6
    
    radius = 12 

    #grid_pt = __approx_to_grid__((x, y, 0))[:2]
    for p in grid:
        if dist((x, y), p) <= radius:
            globals.bad_points.add(p)
            commands.send_vis_command("update-pathfinding-point x:{} y:{} isBlocked:true".format(p[0], p[1]))
    global grid_changed
    grid_changed = True 

def remove_bad_points_around_block(x, y):    
    '''
    x,y is the foot location, for example 1, 1 is in cell 1, 1
    '''
    x = (x * 12) + 6
    y = (y * 12) + 6
    
    radius = 12

    print("REMOVING BLOCK POINTS")
    #grid_pt = __approx_to_grid__((x, y, 0))[:2]
    for p in grid:
        if dist((x, y), p) <= radius:
            if p in globals.bad_points and p not in globals.mothership_bad_points:
                globals.bad_points.remove(p)
                commands.send_vis_command("update-pathfinding-point x:{} y:{} isBlocked:false".format(p[0], p[1]))

    global grid_changed
    grid_changed = True 
