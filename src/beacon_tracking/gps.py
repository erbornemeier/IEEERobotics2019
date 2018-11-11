import cv2
#from servo import servo
from beacon_tracker import beacon_tracker
from threading import Thread

def get_global_position(beacon_l, beacon_r, radius_l, radius_r):

    #get distance between the two beacons
    p1x, p1y = beacon_l
    p2x, p2y = beacon_r
    d = ((p1x - p2x)**2 + (p1y-p2y)**2)**0.5

    # circles intersect zero or infinite times
    if d > radius_l + radius_r or\
       d < abs(radius_l - radius_r) or\
      (d == 0 and radius_l == radius_r):
        return False

    #get legs of triangles
    a = (radius_l**2 - radius_r**2 + d**2) / (2*d)
    h = (radius_l**2 - a**2)**0.5

    #get possible intersection points
    mid_x = p1x + a*(p2x - p1x)/d
    mid_y = p1y + a*(p2y - p1y)/d
    intersect1x = mid_x + h*(p2y - p1y)/d
    intersect2x = mid_x - h*(p2y - p1y)/d
    intersect1y = mid_y - h*(p2x - p1x)/d
    intersect2y = mid_y + h*(p2x - p1x)/d
    intersect1 = (intersect1x, intersect1y)
    intersect2 = (intersect2x, intersect2y)

    #check if the intersection is contained on the board
    #only one point should be contained on the board given a good
    #data set
    def is_on_board(p):
        for c in p:
            if c < 0.0 or c > 96.0:
                return False
        return True
    

    #return intersection 1 if its on the board
    #otherwise just return intersection 2
    if is_on_board(intersect1):
        return intersect1
    return intersect2


def updateServo():
    L_servo.update()

if __name__ == '__main__':
    #set up cameras
    #L_cap, R_cap = cv2.VideoCapture(0), cv2.VideoCapture(1)
    L_cap = cv2.VideoCapture(0)

    #turn off auto-exposure
    L_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    #R_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

    #TODO: remove frame buffer for minimal delay at expense of framerate
    #L_cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
    #R_cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

    #set up beacon trackers
    L_tracker = beacon_tracker()
    #R_tracker = beacon_tracker()

    #set up beacon servos
    L_servo   = servo(pin=18)
    #R_servo   = servo(pin=13)

    updater = Thread(target=updateServo)
    updater.start()

    connected = False
    try:
        arduino = i2c_comm(0x28)
        connected = True
    except:
        pass

    while True:

        _, L_frame = L_cap.read()
        #_, R_frame = R_cap.read()

        L_leds  = L_tracker.get_LED_locations(L_frame, show=True) 
        #R_leds = R_tracker.get_LED_locations(R_frame, show=True) 

        #L_width, R_width = L_cap.get(3), R_cap.get(3)
        L_width = L_cap.get(3)

        if L_leds:
            L_dist = L_tracker.estimate_distance_to_beacon(L_leds)
            print(L_dist)
            #R_dist = R_tracker.estimate_distance_to_beacon(right_leds)

            #L_offset, R_offset = L_width/2 - L_leds[1][0], R_width/2 - R_leds[1][0] 
            L_offset = L_width/2 - L_leds[1][0]

            L_servo.moveDelta(-L_offset/80.0) 
            #R_servo.moveDelta(-R_offset/10.0) 
        cv2.imshow('frame', L_frame)

        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            break
