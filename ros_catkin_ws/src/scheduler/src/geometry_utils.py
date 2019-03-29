import math

def dist(p1, p2):
    return sum((z[1] - z[0])**2 for z in zip(p1[:2], p2[:2]))**0.5

def avg(p1, p2, weight=0.5):
    return tuple([a*weight+b*(1-weight) for a,b in zip(p1[:2],p2[:2])])

def bound_angle(angle):
    if angle > 180:
        angle -= 360
    if angle <= -180:
        angle += 360
    return angle

def pointInEllipse(ellipse_x, ellipse_y, ellipse_rot_degrees, pt_x, pt_y, width, height):
    rot_rad = math.radians(ellipse_rot_degrees)

    c = math.cos(math.radians(-ellipse_rot_degrees))
    s = math.sin(math.radians(-ellipse_rot_degrees))
    x = (pt_x - ellipse_x)
    y = (pt_y - ellipse_y)

    a = ((x*c + y*s) ** 2) 
    b = ((x*s - y*c) ** 2)

    p = (a / ((width/2) ** 2)) + (b / ((height/2) ** 2))
    return p < 1

# Helpful for debugging
# for y in range(12 * 8, -1, -4):
#     for x in range(0, 12 * 8, 4):
#         if pointInEllipse(30, 20, 45, x, y, 24, 20):
#             if y == 0:
#                 print("- ", end ="")
#             elif x == 0:
#                 print("| ", end ="")
#             else:
#                 print("x ", end ="")
#         else:
#             if y == 0:
#                 print("- ", end ="")
#             elif x == 0:
#                 print("| ", end ="")
#             else:
#                 print(". ", end ="")
#     print("")
