import globals
import commands
import time as t
import math

x, y = 0, 1

def __on_line__(l, p3):
    p1, p2 = l
    return p3[x] <= max(p1[x], p2[x]) and p3[x] <= min(p1[x], p2[x]) and\
       p3[y] <= max(p1[y], p2[y]) and p3[y] <= min(p1[y], p2[y])

def __direction__(a, b, c):
    v = (b[y]-a[y])*(c[x]-b[x]) - (b[x]-a[x])*(c[y]-b[y])
    if v == 0:
        return 0
    return 2 if v < 0 else 1

def __ccw__(a, b, c):
    return (c[y]-a[y])*(b[x]-a[x]) > (b[y]-a[y])*(c[x]-a[x])

def __intersection__(l1, l2):
    a, b = l1
    c, d = l2
    return __ccw__(a,c,d) != __ccw__(b,c,d) and __ccw__(a,b,c) != __ccw__(a,b,d)
    '''d1 = __direction__(a, b, c)
    d2 = __direction__(a, b, d)
    d3 = __direction__(c, d, a)
    d4 = __direction__(c, d, b)
    if d1!=d2 and d3!=d4:
        print("TRUE INTERSECTION")
        return True
    if d1==0 and __on_line__(l1, c):
        return True
    if d2==0 and __on_line__(l1, d):
        return True
    if d3==0 and __on_line__(l2, a):
        return True
    if d4==0 and __on_line__(l2, b):
        return True
    return False
    '''

def __collides__(segment, box):

    #connect box points as a series of line segments
    box_lines = zip(box, box[1:] + [box[0]])
    box_lines.append((box[0], box[2]))
    box_lines.append((box[1], box[3]))

    #check if line segment collides with any box segments
    for box_line in box_lines:
        if __intersection__(segment, box_line):
            print("Collided with : {}".format(box_line))
            return True
    return False

def __dist__(p1, p2):
    return sum((z[1] - z[0])**2 for z in zip(p1, p2))**0.5

def go_to_point(current_pos, target_pos, approach_dist=0):
    dx, dy = target_pos[0]-current_pos[0], target_pos[1]-current_pos[1]
    forward_dist = (dx**2 + dy**2)**0.5 - approach_dist 
    forward_dist = max(0, forward_dist)
    turn_angle = (math.atan2(dy, dx) * 180.0/3.14159) - current_pos[2] 
    if turn_angle > 180:
        turn_angle -= 360
    if turn_angle < -180:
        turn_angle += 360
    #print("TURNING: {} THEN DRIVING {}".format(turn_angle, forward_dist))
    if abs(turn_angle) > 0.1:
        commands.send_drive_turn_command(turn_angle)
        t.sleep(5)
    if forward_dist > 0.1:
        commands.send_drive_forward_command(forward_dist)
        t.sleep(6)


def drive_safely(current_pos, target_pos, approach_dist=0):
    current_pos_xy = current_pos[:2]

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

    #if a collision of mothership is predicted
    #TODO too close to mothership to make a 2 segment move
    if (__collides__(current_to_target, mothership_box)):
        print("FINDING PATH TO TARGET")
        #find path to target
        for b in mothership_pts:
            if not (__collides__( (current_pos_xy, b), mothership_box) or 
                    __collides__( (b, target_pos)    , mothership_box)):
                return b
                #go_to_point(cb[0], cb[1])
                #go_to_point(bt[0], bt[1], approach_dist)
    return None
