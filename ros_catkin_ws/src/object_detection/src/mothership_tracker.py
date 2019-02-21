#!/usr/bin/python
import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import Image
from object_detection.srv import *
from cv_bridge import CvBridge
bridge = CvBridge()

class mothership_tracker:
    
    LED_MASK = np.array([[ 35, 100, 150],
                          [80, 255, 255]])
    LINE_THRESHOLD = 200 
    LINE_LEN_THRESH = 200 
    PIXEL2INCH = 1.0 / 262.0 
    
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

        #return in order top, middle, bottom
        return leds


    def estimate_distance_to_mothership(self, leds):
        top, _, bottom = leds
        #if the top and bottom were found
        if top is not None and bottom is not None:
            #get distance between them in pixels
            pixels =  ((top[0] - bottom[0]) ** 2 + (top[1] - bottom[1])**2) ** 0.5
            #convert to inches
            if pixels > 4:
                inches = 12 / (pixels * self.PIXEL2INCH)
                return inches
        return False


    def __find_potential_LEDs__(self, frame, show=False):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, self.LED_MASK[0], self.LED_MASK[1])
        mask = cv2.bitwise_or(mask, mask_green)
        #dilate image to expand contours
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)) 
        mask = cv2.erode(mask, kernel)
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
            if show:
                cv2.circle(frame, (int(cx), int(cy)), 7, (0,0,255), -1)
            return (cx, cy)
        return False


    def __find_LED_line__(self, frame, centers, show=False):
        top, middle, bottom = None, None, None 
        smallestDiff = float('inf')
        for c_top in centers:
            for c_bottom in centers:
                if c_top != c_bottom:
                    if (c_top[0] - c_bottom[0])**2 + (c_top[1] - c_bottom[1])**2 > self.LINE_LEN_THRESH:
                        midpt = ((c_top[0] + c_bottom[0]) // 2, (c_top[1] + c_bottom[1]) // 2)
                        for c_middle in centers:
                            if c_middle not in [c_top, c_bottom]:
                                middle_diff = (c_middle[0] - midpt[0])**2 + (c_middle[1] - midpt[1])**2
                                if middle_diff < smallestDiff:
                                    smallestDiff = middle_diff
                                    top, middle, bottom = c_top, c_middle, c_bottom 
        if top is not None and smallestDiff < self.LINE_THRESHOLD:
            if show:
                cv2.line(frame, (int(top[0]), int(top[1])), (int(bottom[0]), int(bottom[1])), (255,255,0), 5)
            if top[0] > bottom[0]:
                temp = top
                top = bottom
                bottom = temp
            return (top, middle, bottom)
        return False

def image_recieved(data):
    global img
    global width
    global height
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    width, height = len(img[0]), len(img)

tracker = mothership_tracker()
def get_line_params(_):
    msg = Mothership()
    try:
        left, mid, right = tracker.get_LED_locations(img)
        msg.x = mid[0] / width
        msg.y = mid[1] / height
        dx, dy = right[0] - left[0], right[1] - left[1]
        msg.theta = math.atan2(dy, dx)
        return msg 
    except Exception as e:
        print(e)
        msg.x, msg.y = -1, -1
        return msg
    

rospy.init_node("mothership_tracker")
rospy.Subscriber("camera_image", Image, image_recieved)
block_pos_srv = rospy.Service("mothership", Mothership, get_line_params)
rospy.spin()

