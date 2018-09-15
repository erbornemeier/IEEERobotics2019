#!/usr/bin/python3


#Cross Detection
#Returns the difference between the center of the camera and the center of the 
#cross feature as well as the difference between the intended orientation and
#the goal orientation

import cv2
import numpy as np
from math import atan2
from geometryHelpers import *

cap = cv2.VideoCapture(1)

kernel = np.ones((5,5) , np.uint8)

CENTER = [i/2 for i in cap.read()[1].shape[:2]]

while True:
    
    #take in image and clean it
    _, img = cap.read()
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur (img_gray, (5,5), 0)
    #threshold the image to binary
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(blur, 135, 255,cv2.THRESH_BINARY_INV)
    thresh = cv2.erode(thresh, kernel, iterations=1)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    #get the contours of the image
   
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (120,120))

    center_find = thresh
    center_find = cv2.erode(center_find, kernel2, iterations=1)
  
    _, contours, hierarchy = cv2.findContours(center_find, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[np.argmax(np.array([cv2.contourArea(cnt) for cnt in contours]), axis=0)]
    M = cv2.moments(cnt)
    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    cv2.circle(img, (cx,cy), 5, [0,0,255], -1)

    #if contours exist
    if contours:
        ###################
        #   CONTOURS AND  #
        #     DEFECTS     #
        ###################
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #get the maximum contour
        cnt = contours[np.argmax(np.array([cv2.contourArea(cnt) for cnt in contours]), axis=0)]
        #get convex hull of max contour
        hull = cv2.convexHull(cnt,returnPoints = False)
        #get convexity defects between contour and convex hull
        defects = cv2.convexityDefects(cnt,hull)
        
        top_defects = defects[defects[:,0][:,3].argsort()[-4:]]
      
        pts = []
        for i in range(top_defects.shape[0]):
            #get start, end, defect pt, dist
            s,e,f,d = top_defects[i,0]
            start = tuple(cnt[s][0])
            end = tuple(cnt[e][0])
            #append relevant hull points 
            far = tuple(cnt[f][0])
            btwn = angle_between(start, far, end)
            pts.append((far, btwn*180/3.14159))
            cv2.line(img, start, end, [0,255,0],2)
             
        pts.sort(key=lambda x: x[1])
        print(pts)
        pts = pts[:2]
        for p in pts:
            cv2.circle(img, p[0], 5, [255,0,0], -1)
        mid = midpoint(tuple(pts[0][0]), tuple(pts[1][0]), toInt=True)
        end1 = ((mid[0] - cx)*1000 + cx, (mid[1] -cy)*1000 + cy)
        end2 = ((mid[0] - cx)*-1000 + cx, (mid[1] -cy)*-1000 + cy)
        cv2.line(img, end1, end2, [255,0,0], 2)            
        ''' 
        pts = [tuple(pt) for pt in cv2.boxPoints(cv2.minAreaRect(cnt))]
        
        
        for pt in pts:
            cv2.circle(img,tuple(pt),5,[255,0,0],-1)
        print(pts)
        
        
        mid_pts = []
        while pts:
            first = pts.pop()
            if not pts:
                break
            second = min(pts, key=lambda p: length_squared(p, first) )
            pts.remove(second) 
            mid_pts.append(midpoint(first, second, toInt = True)) 
            cv2.circle(img,mid_pts[-1],5,[0,255,0],-1)
        if len(mid_pts) > 1:
            cv2.line(img, mid_pts[0], mid_pts[1], [0,255,0], 2)

        #cv2.drawContours(img, [np.int0(pts)], 0, (0, 0, 255), 2)
        '''       
    cv2.imshow("cap", img);
    cv2.imshow("test", center_find)
    if cv2.waitKey(1) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break;
    
