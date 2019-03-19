#!/usr/bin/python3
import cv2
import numpy as np
from itertools import permutations
import time as t


class beacon_tracker:
    
    LED_MASK = np.array([[ 20, 0, 250],
                          [100, 255, 255]])
    MAX_ERROR = 0.25
    
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
        #mask = cv2.erode(mask, kernel)
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
                pass
                #cv2.circle(frame, (int(cx), int(cy)), 7, (0,0,255), -1)
            return (cx, cy)
        return False


    def __dist__(self, p1, p2):
        return sum((a[0] - a[1])**2 for a in zip(p1,p2))**0.5

    def __avg__(self, p1, p2, weight=0.5):
        return tuple([p1[i]*weight + p2[i]*(1-weight) for i in range(len(p1))])

    def __find_LED_line__(self, frame, centers, show=False):

        for quad in permutations(centers, 4):
            l, lm, rm, r = quad
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



#TEST OF BEACON TRACKER
if __name__ == '__main__':

    cap = cv2.VideoCapture(1)

    tracker = beacon_tracker()

    width, height = cap.get(3), cap.get(4)

    while True:
        #get next frame
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0), fx=0.5, fy=0.5)

        #unsuccessful frame capture
        if not ret:
            raise IOError("Frame could not be read")

        leds = tracker.get_LED_locations(frame, show=True)
        cv2.imshow('Beacon Tracker Test', frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

