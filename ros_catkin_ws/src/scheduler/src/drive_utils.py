from __future__ import print_function
import globals
from pypaths import astar
import commands
from geometry_utils import *
import time as t
import math
import rospy
from geometry_msgs.msg import Pose2D
import RPi.GPIO as RPIO

# setup ack pin
ack_pin = 40
stop_pin = 38
start_pin = 36
RPIO.setmode(RPIO.BOARD)
RPIO.setup(ack_pin, RPIO.IN, pull_up_down=RPIO.PUD_UP)
RPIO.setup(stop_pin, RPIO.OUT)
RPIO.output(stop_pin, RPIO.LOW)
RPIO.setup(start_pin, RPIO.IN, pull_up_down=RPIO.PUD_DOWN)

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
    dots = 0
    print("")
    while robot_x == -1 and t.time() - start_time < TIMEOUT:
        print('\r                             ', end="")
        print('\rwaiting for pose update{}'.format('.'*dots), end="")
        t.sleep(0.05)
        dots = (dots + 1) % 4
    print("")

def wait_for_msg_received():
    start_time = t.time()
    while RPIO.input(ack_pin) == RPIO.LOW and t.time() - start_time < TIMEOUT:
        print("Waiting for busy pin to be set")
        t.sleep(0.05)
    if t.time() - start_time >= TIMEOUT:
        return False
    return True

def wait_for_start_button():
    while RPIO.input(start_pin) == RPIO.LOW:
        pass

def wait_for_pose_change():
    dots = 0
    while RPIO.input(ack_pin) == RPIO.HIGH:
        print('\r                             ', end="")
        print('\rwaiting for pose change{}'.format('.'*dots), end="")
        t.sleep(0.05)
        dots = (dots + 1) % 4
    wait_for_pose_update()

# PATH FINDING
RESOLUTION = 4
MARGIN = 8
FIRST_POINT_IN = int(math.ceil(MARGIN/float(RESOLUTION))*RESOLUTION)
print(FIRST_POINT_IN)

def __custom_neighbors__( height, width ):
    def func( coord ):
        '''
        neighbor_list = [(coord[0]+i,coord[1]+j) 
                            for i in range(-RESOLUTION, RESOLUTION+1, RESOLUTION)
                            for j in range(-RESOLUTION, RESOLUTION+1, RESOLUTION) ]
        return [ c for c in neighbor_list
                 if c != coord
                 and c not in globals.bad_points
                 and c[0] >= MARGIN and c[0] <= width-MARGIN
                 and c[1] >= MARGIN and c[1] <= height-MARGIN ]
        '''
        neighbor_list = [(coord[0], coord[1], coord[2]+angle) \
                        for angle in range(-135, 181, 45)]
        next_x = coord[0] + math.cos(math.radians(coord[2]))*\
                            (1 if coord[2]%90==0 else 2**0.5)*RESOLUTION
        next_y = coord[1] + math.sin(math.radians(coord[2]))*\
                            (1 if coord[2]%90==0 else 2**0.5)*RESOLUTION
        neighbor_list.append((int(round(next_x)), int(round(next_y)), coord[2]))

        return [ c for c in neighbor_list
                 if c != coord
                 and c[:2] not in globals.bad_points
                 and c[0] >= MARGIN and c[0] <= width-MARGIN
                 and c[1] >= MARGIN and c[1] <= height-MARGIN ]

    return func

def __custom_cost__(a, b):
    if a[2] != b[2]:
        return 7 # 5 and 10 was decent
    else:
        return 1

finder = astar.pathfinder(neighbors=__custom_neighbors__(12*8, 12*8),
                    distance=astar.absolute_distance,
                    cost=__custom_cost__)
def __optimize_path__(path):
    #optimized_path = [path[0]]
    optimized_path = []
    for i, p in enumerate(path[1:-1]):
        if path[i][:2] == p[:2]:
            continue
        
        if p[:2] != avg(path[i], path[i+2]):
            optimized_path.append(p)
    #optimized_path.append(path[-1])
    return optimized_path

grid = [(i,j) for i in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)\
              for j in range(FIRST_POINT_IN, 12*8-FIRST_POINT_IN+1, RESOLUTION)]
def __approx_to_grid__(pt):
    closest = sorted(grid, key=lambda x: dist(x, pt[:2]))
    snap_angle = int(round(pt[2]/float(45)))*45
    for p in closest:
        if p not in globals.bad_points:
            return (p[0], p[1], snap_angle)
    print("COULD NOT APPROXIMATE POINT TO A VALID POINT, CHOSE CLOSEST")
    return tuple([closest[0]] + [snap_angle])

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

def stop():
    RPIO.output(stop_pin, RPIO.HIGH)
    t.sleep(0.25)
    RPIO.output(stop_pin, RPIO.LOW)

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
    robot_pos = (robot_x, robot_y, robot_theta)
    if approach_dist != 0:
        to_pt = get_approach_point(robot_pos, to_pt, approach_dist)
        to_pt = __approx_to_grid__((to_pt[0], to_pt[1], 0))
    to_pt = (to_pt[0], to_pt[1], 0)
    path = __get_path__(robot_pos, to_pt)

    if approach_dist != 0:
        path = path[:-1]

    print("PATH: {}".format(path))
    #TODO change to instructions to send to arduino
    for p in path:
        drive_to_point(p)

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

def remove_bad_points_around_block(x, y):    
    '''
    x,y is the foot location, for example 1, 1 is in cell 1, 1
    '''
    x = (x * 12) + 6
    y = (y * 12) + 6
    
    radius = 12

    #grid_pt = __approx_to_grid__((x, y, 0))[:2]
    for p in grid:
        if dist((x, y), p) <= radius:
            if p in globals.bad_points:
                globals.bad_points.remove(p)
