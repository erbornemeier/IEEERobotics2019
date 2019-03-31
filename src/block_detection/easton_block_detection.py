import cv2
import numpy as np

black_lower = np.array([0,0,0])                                               
black_upper = np.array([255,255,80])                                           
kernel = np.ones((5,5),np.uint8)                                                
kernel2 = np.ones((5,5),np.uint8) 

cap = cv2.VideoCapture(1)

while True:
    _, img = cap.read()
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                              
                                                                            
    #orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    #non_orange_mask = cv2.bitwise_not(orange_mask)
    #non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_OPEN, kernel)
    #non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_CLOSE, kernel)

    #white_mask = cv2.inRange(hsv, white_lower, white_upper)
    #white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
    #white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.bitwise_and(white_mask, non_orange_mask)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2)

    thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN, cv2.THRESH_BINARY_INV, 11, 2)

    '''
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours) > 0):
        block = max(contours, key=cv2.contourArea)
        M = cv2.moments(block)
        screenX = int(M["m10"] / M["m00"])
        screenY = int(M["m01"] / M["m00"])

        cv2.imshow('white_mask', white_mask)
        cv2.imshow('non-orange-mask', non_orange_mask)
        cv2.imshow('mask', mask)
    '''

    cv2.imshow('thresh', thresh)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
        
