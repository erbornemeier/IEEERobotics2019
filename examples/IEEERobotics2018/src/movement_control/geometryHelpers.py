
import numpy as np
import math

def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def seg_intersect(a1,a2, b1,b2, toInt = False) :
    a1,a2,b1,b2 = np.array(a1), np.array(a2), np.array(b1), np.array(b2)
    da = a2 - a1 
    db = b2 - b1 
    dp = a1 - b1 
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )

    if not toInt:
        return (num / denom.astype(float))*db + b1
    else:
        return ((num / denom.astype(float))*db + b1).astype(np.int32)

def projection(a, b, p, toInt = False):
    #get the length of the hull line
    lineLen = (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    #distance down the projection line
    t = max(0, min(1, np.dot(np.array(p) - np.array(a),\
	       np.array(b) - np.array(a))  \
	       / lineLen )) 
    #get the projection line point
    proj = np.array(a) + t * (np.array(b) - np.array(a))
    if not toInt:
        return proj
    else:
        return proj.astype(np.int32)

def length_squared(a,b, toInt = False):
    if not toInt:
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    else:
        return int((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def length(a,b, toInt = False):
    if not toInt:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) **0.5
    else:
        return int(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)


def midpoint(a, b, toInt = False):
    if not toInt:
        return ((a[0] + b[0])/2,(a[1] + b[1])/2)
    else:
        return (int((a[0] + b[0])/2),int((a[1] + b[1])/2) )


def angle_between(a, b, c, toInt = False):
    vec1 = (a[0] - b[0], a[1] - b[1])
    vec2 = (c[0] - b[0], c[1] - b[1])
    if not toInt:
        return math.acos(np.dot(vec1, vec2) / (length(a,b)*length(c,b)))
    else:
        return int(math.acos(np.dot(vec1, vec2) / (length(a,b)*length(c,b))))

def rotate_around(origin, point, deg, toInt=False):
    ox, oy = origin
    px, py = point
    rads = deg * math.pi/180.0
    rx = ox + math.cos(rads) * (px - ox) - math.sin(rads) * (py - oy)
    ry = oy + math.sin(rads) * (px - ox) + math.cos(rads) * (py - oy)
    if not toInt:
        return (rx, ry) 
    else:
        return (int(rx), int(ry))
