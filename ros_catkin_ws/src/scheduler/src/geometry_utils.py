import math

def pointInEllipse(ellipse_x, ellipse_y, ellipse_rot_degrees, pt_x, pt_y, width, height):
    rot_rad = math.radians(ellipse_rot_degrees)

    c = math.cos(math.radians(-ellipse_rot_degrees))
    s = math.sin(math.radians(-ellipse_rot_degrees))
    x = (pt_x - ellipse_x)
    y = (pt_y - ellipse_y)

    p = (((x*c + y*s) ** 2) / (width ** 2)) + (((x*s-y*c) ** 2) / (height ** 2)) 
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