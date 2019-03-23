import globals
import commands
import time as t
import math

def __intersection__(line1, line2):
    x=0
    y=1
    a,b = line1
    c,d = line2
    den  = (b[x] - a[x])*(d[y] - c[y]) - (b[y] - a[y])*(d[x] - c[x])
    num1 = (a[y] - c[y])*(d[x] - c[x]) - (a[x] - c[x])*(d[y] - c[y])
    num2 = (a[y] - c[y])*(b[x] - a[x]) - (a[x] - c[x])*(b[y] - a[y])
    if den==0:
        return not (num1==0 and num2==0)
    r = num1/den
    s = num2/den
    return (r>0 and r<1) and (s>0 and s<1)

def __collides__(segment, box):

    #connect box points as a series of line segments
    box_lines = zip(box, box[1:] + [box[0]])
    box_lines.append((box[0], box[2]))
    box_lines.append((box[1], box[3]))

    #check if line segment collides with any box segments
    for box_line in box_lines:
        if __intersection__(segment, box_line):
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
    mothership_box = [(globals.abc_x, globals.abc_y),
                      (globals.af_x, globals.af_y),
                      (globals.def_x, globals.def_y),
                      (globals.cd_x, globals.cd_y)]

    #if a collision of mothership is predicted
    #TODO too close to mothership to make a 2 segment move
    if (__collides__(current_to_target, mothership_box)):
        #find path to target
        for b in mothership_box:
            if not (__collides__((current_pos_xy, b), mothership_box) or __collides__((b, target_pos), mothership_box)):
                return b
                #go_to_point(cb[0], cb[1])
                #go_to_point(bt[0], bt[1], approach_dist)
    return None
