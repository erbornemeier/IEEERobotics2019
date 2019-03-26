#!/usr/bin/python
import cv2
import numpy as np
import math
from itertools import permutations
import rospy
from sensor_msgs.msg import Image
from object_detection.srv import *
from cv_bridge import CvBridge
bridge = CvBridge()

class mothership_tracker:
    
    LED_MASK = np.array([[ 20,   0,   250],
                          [100, 255, 255]])
    MAX_ERROR = 0.18
     
    def get_LED_locations(self, frame, show=False):
        #get potential LED locations
        contours = self.__find_potential_LEDs__(frame, show=show)
        if not contours:
            return False
       
        #find the center of all potential LEDs
        centers = []
        for c in contours:
           center = self.__get_center_of_contour__(frame, c, show=show) 
           if center:
               centers.append(center)

        #find the three LEDs that best form a straight line
        leds = self.__find_LED_line__(frame, centers, show=show)
        if not leds:
            return False

        return leds

    def __find_potential_LEDs__(self, frame, show=False):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)) 
        mask = cv2.dilate(mask, kernel)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, self.LED_MASK[0], self.LED_MASK[1])
        mask = cv2.bitwise_and(mask, mask_green)
        #dilate image to expand contours
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11)) 
        mask = cv2.dilate(mask, kernel)
        #find contours
        _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #show mask and contours
        if show:
            cv2.imshow('Potential LEDS', mask)
            cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        return contours 


    def __get_center_of_contour__(self, frame, contour, show=False):
        # get moments of contour
        M = cv2.moments(contour)
        #if large enough to have center
        if M['m00'] > 0:
            #get center point
            cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
            return (cx, cy)
        return False


    def __dist__(self, p1, p2):
        return sum((a[0] - a[1])**2 for a in zip(p1,p2))**0.5

    def __avg__(self, p1, p2, weight=0.5):
        return tuple([p1[i]*weight + p2[i]*(1-weight) for i in range(len(p1))])

    def __find_LED_line__(self, frame, centers, show=False):

        for quad in permutations(centers, 4):
            l, lm, rm, r = quad
            if l[0] > r[0]:
                continue
            line_dist = self.__dist__(l,r)
            l_lm_dist = self.__dist__(l, lm)
            lm_rm_dist = self.__dist__(lm, rm)
            rm_r_dist = self.__dist__(rm, r) 
            #skip of not resembling a line
            if max(line_dist, l_lm_dist, lm_rm_dist, rm_r_dist) != line_dist:
                continue
            #get expected center light locations
            lm_ideal = self.__avg__(l,r,0.75)
            rm_ideal = self.__avg__(l,r,0.25)
            #check middle light locations
            error = self.__dist__(lm, lm_ideal) + self.__dist__(rm, rm_ideal)
            #check straightness of line
            error += abs(line_dist - l_lm_dist - lm_rm_dist - rm_r_dist)
            #normalize
            error /= line_dist

            if error < self.MAX_ERROR:
                if show:
                    for p in quad:
                        cv2.circle(frame, tuple(map(int, p)), 3, (0,0,255), 3)
                    cv2.line(frame, tuple(map(int, l)), tuple(map(int, r)), (255,0,0), 3)
                    print(error)
                return quad 
        return False

def image_recieved(data):
    global img
    global width
    global height
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    width, height = len(img[0]), len(img)

tracker = mothership_tracker()
def get_line_params(_):
    msg = MothershipResponse()
    try:
        leds = tracker.get_LED_locations(img)
        if not leds:
            raise Exception("Mothership not found.")
        left, mid_l, mid_r, right = leds 
        midpt = tracker.__avg__(mid_l, mid_r)
        msg.x, msg.y = midpt[0]/float(width), midpt[1] / float(height)
        dx, dy = right[0] - left[0], right[1] - left[1]
        msg.theta = math.atan2(dy, dx) * 180.0/math.pi
        return msg 
    except Exception as e:
        print(e)
        msg.x, msg.y = -1, -1
        return msg
    

rospy.init_node("mothership_tracker")
rospy.Subscriber("camera_image", Image, image_recieved)
block_pos_srv = rospy.Service("mothership", Mothership, get_line_params)
rospy.spin()

