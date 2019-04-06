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
    MAX_THREE_ERROR = 0.05
     
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
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1,1)) 
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

    def __three_line__(self, frame, centers, show=False):
        
        min_error = float('inf')
        quad_min = None
        for tri in permutations(centers, 3):
            l, m, r = tri
            if r[0] - l[0] > 10:
                continue
            line_dist = self.__dist__(l,r)
            l_m_dist  = self.__dist__(l,m)
            m_r_dist  = self.__dist__(m,r)
            if max(line_dist, l_m_dist, m_r_dist) != line_dist:
                continue
            m_ideal_l = self.__avg__(l,r, 0.75)
            error_m_ml = self.__dist__(m, m_ideal_l)
            m_ideal_m = self.__avg__(l,r, 0.50)
            error_m_mm = self.__dist__(m, m_ideal_m)
            m_ideal_r = self.__avg__(l,r, 0.25)
            error_m_mr = self.__dist__(m, m_ideal_r)

            mid_pt = m_ideal_m
            #better left than right
            if error_m_ml < error_m_mr: 
                #better than mid
                if error_m_ml < error_m_mm:
                    mid_pt = m_ideal_l
            #better right than left 
            else:
                #better than mid
                if error_m_mr < error_m_mm:
                    mid_pt = m_ideal_r

            if mid_pt == m_ideal_l:
                quad = (l, m, m_ideal_r, r)
            elif mid_pt == m_ideal_r:
                quad = (l, m_ideal_l, m, r)
            else:
                quad = (l, m, r)


            error = self.__dist__(m, mid_pt)
            error += abs(line_dist - l_m_dist - m_r_dist)
            error /= float(line_dist)

            if error < self.MAX_THREE_ERROR:
                print ("THREE LINE ERROR: {}".format(error))
                return quad
        return False
            

    def __find_LED_line__(self, frame, centers, show=False):

        for quad in permutations(centers, 4):
            l, lm, rm, r = quad
            if r[0] - l[0] > 10:
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
        return self.__three_line__(frame, centers)

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
        if len(leds) == 4:
            left, mid_l, mid_r, right = leds 
            midpt = tracker.__avg__(mid_l, mid_r)
        else:
            left, mid, right = leds
            midpt = mid
        msg.x, msg.y = midpt[0]/float(width), midpt[1] / float(height)
        dx, dy = right[0] - left[0], right[1] - left[1]
        msg.theta = math.atan2(dy, dx) * 180.0/math.pi
        return msg 
    except Exception as e:
        #print(e)
        msg.x, msg.y = -1, -1
        return msg

orange_lower = np.array([4, 50, 100])
orange_upper = np.array([18, 255, 255])
kernel = np.ones((3,3),np.uint8)                                                

def find_biggest_orange(_):
    msg = MothershipResponse()
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
       	orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    	orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)

	_, contours, hier = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) > 0:
	    maxIndex = np.argmax([cv2.contourArea(c) for c in contours])
	    interiors = []
	    AREA_THRESH = 100
	    for i, h in enumerate(hier[0]):
		if h[3] == maxIndex:
		    if cv2.contourArea(contours[i]) > AREA_THRESH:
			interiors.append(contours[i])
	    if len(interiors) > 0:
		maxContour = max(interiors, key = cv2.contourArea)  
		msg.x = cv2.contourArea(maxContour)
		return msg 
	raise Exception('no contour found')
    except Exception as e:
        #print(e)
        msg.x, msg.y = -1, -1
        return msg

rospy.init_node("mothership_tracker")
rospy.Subscriber("camera_image", Image, image_recieved)
block_pos_srv = rospy.Service("mothership", Mothership, get_line_params)
orange_pos_srv = rospy.Service("big_orange", Mothership, find_biggest_orange)
rospy.spin()

