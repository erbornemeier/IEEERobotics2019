#!/usr/bin/python

import cv2
import numpy as np
import rospy
import traceback
from sensor_msgs.msg import Image
from object_detection.srv import *
from cv_bridge import CvBridge
bridge = CvBridge()


                                                                                
diff_weight = 1                                                                 
aspect_weight = 20                                                              
min_goodness = 12     

def image_recieved(data):
    global img
    global width
    global height
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    width, height = len(img[0]), len(img)

def get_block_pos_close(_):
    orange_lower = np.array([4,100,0])                                               
    orange_upper = np.array([18,255,255])                                              
    white_lower = np.array([0,0,200])                                                  
    white_upper = np.array([255,255,255])                                              
    kernel = np.ones((5,5),np.uint8)                                                   
    kernel2 = np.ones((5,5),np.uint8)                                               
    response = BlockResponse()
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
        non_orange_mask = cv2.bitwise_not(orange_mask)
        non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_OPEN, kernel)
        non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_CLOSE, kernel)

        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.bitwise_and(white_mask, non_orange_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) > 0):
            block = max(contours, key=cv2.contourArea)
            M = cv2.moments(block)
            screenX = int(M["m10"] / M["m00"])
            screenY = int(M["m01"] / M["m00"])
            response.x, response.y = screenX / float(width), screenY / float(height)
            return response

    except Exception as e:
        #print (e)
        response.x = -1
        response.y = -1
        return response
'''
def average_contrast(img, contour, debug=False):
    x, y, w, h = cv2.boundingRect(contour)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    black = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.drawContours(black, [contour], 0, (255,255,255), thickness=cv2.FILLED)
    cropped = cv2.bitwise_and(gray, black)
    if cropped.size == 0:
        return 0
    avg_pixel_value = np.sum(cropped) / float(np.count_nonzero(cropped))

    diff = cv2.bitwise_and(black.astype(np.float64), np.abs(cropped - avg_pixel_value))
    pixel_diffs = np.sum(diff)
    avg_pixel_diff = pixel_diffs / float(np.count_nonzero(diff))
    aspect_error = max(w/float(h), h/float(w))-1
    goodness =  avg_pixel_diff*diff_weight - aspect_error*aspect_weight 
    if debug:
        cv2.putText(img, str(round(goodness, 3)), (x+w//2, y+h//2),\
                 cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255), 2)
    return goodness

def get_block_contour(img, debug=False):
    orange_lower = np.array([4,100,100])                                               
    orange_upper = np.array([18,255,240])                                              
    white_lower = np.array([0,0,140])                                                  
    white_upper = np.array([255,255,255])                                              
    kernel = np.ones((2,2),np.uint8)                                                   
    kernel2 = np.ones((2,2),np.uint8)                                               
    kernel3 = np.ones((5,5),np.uint8)                                               

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                              
                                                                            
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
    _, contours, hier = cv2.findContours(orange_mask, cv2.RETR_TREE, 
                                                      cv2.CHAIN_APPROX_SIMPLE) 
    interiors = []
    maxIndex = np.argmax([cv2.contourArea(c) for c in contours])
    AREA_THRESH = 100 
    for i, h in enumerate(hier[0]):
        if h[3] == maxIndex:
            if cv2.contourArea(contours[i]) > AREA_THRESH:
                interiors.append(contours[i])

    if len(interiors) > 0:
        goodness_values = [(i, average_contrast(img, i, debug=debug)) for i in interiors]
        max_contrast = max(goodness_values, key=lambda x: x[1])
        if debug:
            cv2.imshow('mask', orange_mask)
        if max_contrast[1] > min_goodness:
            print("INTERIOR {}".format(max_contrast[1]))
            return max_contrast[0]

    non_orange_mask = cv2.bitwise_not(orange_mask)
    _, contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_EXTERNAL, 
                                                  cv2.CHAIN_APPROX_SIMPLE) 
    goodness_values = [(c, average_contrast(img, c, debug=debug)) for c in contours]
    max_contrast = max(goodness_values, key=lambda x: x[1])
    if debug:
        cv2.imshow('no_mask', non_orange_mask)
    if max_contrast[1] > min_goodness:
        print(max_contrast[1])
        return max_contrast[0]
    return None

'''
def get_block_contour(img, debug=False):                                        
    orange_lower = np.array([0,60,0])                                              
    orange_upper = np.array([22,255,255])                                               
    white_lower = np.array([0,0,145])                                                   
    white_upper = np.array([255,255,255])                                               
    kernel = np.ones((2,2),np.uint8)                                                    
    kernel2 = np.ones((2,2),np.uint8)                                           
    kernel3 = np.ones((5,5),np.uint8)                                           
                                                                                
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                                   
                                                                                 
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)                  
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)        
                                                                                
    _, o, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    board = np.zeros(img.shape, dtype=np.uint8)                                 
    cv2.drawContours(board, [max(o, key=cv2.contourArea)], 0, (255,255,255), \
                                thickness=cv2.FILLED)                           
    board = cv2.bitwise_and(img, board)                                         
                                                                                
    hsv = cv2.cvtColor(board, cv2.COLOR_BGR2HSV)                                
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)                  
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)        
    non_orange_mask = cv2.bitwise_not(orange_mask)                              
                                                                                
    white_mask = cv2.inRange(hsv, white_lower, white_upper)                     
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)          
    mask = cv2.bitwise_and(non_orange_mask, white_mask)                         
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)                      
    mask = cv2.dilate(mask, kernel, iterations=2)                               
                                                                                
    #cv2.imshow('mask', mask)                                                    
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,                  
                                                      cv2.CHAIN_APPROX_SIMPLE)  
    AREA_THRESH = 500                                                           
    contours = list(filter(lambda c: cv2.contourArea(c) > AREA_THRESH, contours))
    isolated = np.zeros(img.shape[:2], dtype=np.uint8)                          
    cv2.drawContours(isolated, contours, -1, (255,255,255), thickness=cv2.FILLED)
    _, diff = cv2.threshold(np.absolute(isolated-mask), 128, 255, cv2.THRESH_BINARY)
    #diff = cv2.morphologyEx(diff, cv2.MORPH_CLOSE, kernel3)                    
    diff = cv2.dilate(diff, kernel3)                                            
    #cv2.imshow('iso', diff)                                                     
                                                                                
    _, letters, _ = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #dont touch left, right, or top
    def no_edge_touch(img, contour):                                            
        h,w = img.shape[:2]                                                     
        x, y, ww, hh = cv2.boundingRect(contour)                                
        if x <= 0 or x+ww >= w or y+hh >= h:                              
            return False                                                        
        return True                                                             
    letters = list(filter(lambda c: no_edge_touch(img, c), letters))            
                                                                                
    if len(letters) > 0:                                                        
        letter = max(letters, key=cv2.contourArea)                              
        print(cv2.contourArea(letter))
        return letter                                                           
    return False             

def get_block_pos(_):
    response = BlockResponse()
    try:
        block = get_block_contour(img)
        if block is None:
            raise Exception('no block found')
        M = cv2.moments(block)
        screenX = int(M["m10"] / M["m00"])
        screenY = int(M["m01"] / M["m00"])
        response.x, response.y = screenX / float(width), screenY / float(height)
        return response

    except Exception as e:
        traceback.print_exc()
        print (e)
        response.x = -1
        response.y = -1
        return False 

rospy.init_node("block_position_find")
rospy.Subscriber("camera_image", Image, image_recieved)
block_pos_srv = rospy.Service("block_pos", Block, get_block_pos)
block_pos_close_srv = rospy.Service("block_pos_close", Block, get_block_pos_close)
rospy.spin()
