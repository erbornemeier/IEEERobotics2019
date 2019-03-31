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

## Actually point in rectangle but didn't feel like changing the name
def pointInEllipse(rect_x, rect_y, mothership_theta, pt_x, pt_y, rect_width, rect_height):
    s = math.sin(math.radians(-mothership_theta))
    c = math.cos(math.radians(-mothership_theta))

    x1 = pt_x - rect_x
    y1 = pt_y - rect_y

    x = x1*c - y1*s + rect_x
    y = x1*s + y1*c + rect_y

    return x >= rect_x - rect_width/2 and x <= rect_x + rect_width/2 and y >= rect_y - rect_height/2 and y <= rect_y + rect_height/2
    # rot_rad = math.radians(ellipse_rot_degrees)

    # c = math.cos(math.radians(-ellipse_rot_degrees))
    # s = math.sin(math.radians(-ellipse_rot_degrees))
    # x = (pt_x - ellipse_x)
    # y = (pt_y - ellipse_y)

    # a = ((x*c + y*s) ** 2) 
    # b = ((x*s - y*c) ** 2)

    # p = (a / ((width/2) ** 2)) + (b / ((height/2) ** 2))
    # return p < 1

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
