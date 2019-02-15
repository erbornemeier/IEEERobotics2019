import cv2
import numpy as np

cap = cv2.VideoCapture(1)

orange_lower = np.array([4,100,9])
orange_upper = np.array([18,255,255])
white_lower = np.array([0,0,230])
white_upper = np.array([255,255,255])
kernel = np.ones((7,7),np.uint8)

while True:
    _, frame = cap.read()

    #threshold the image
    thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.GaussianBlur(thresh, (5,5), 0)
    thresh = cv2.adaptiveThreshold(thresh, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #get the orange contours in the image
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #get the non orange contours in the image
    non_orange_mask = cv2.bitwise_not(orange_mask)
    non_orange_contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    white_mask = cv2.inRange(hsv, white_lower, white_upper)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    #get the white contours in the image
    white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #invert and get the black contours in the image
    thresh2 = cv2.bitwise_not(thresh)
    black_contours, _ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    #find nonorange contours within orange ones
    for no in white_contours:
        no_x, no_y, no_dx, no_dy = cv2.boundingRect(no)
        for o in orange_contours:
            o_x, o_y, o_dx, o_dy = cv2.boundingRect(o)
            if no_dx > 50 and no_dy > 50 and  no_x > o_x and no_x + no_dx < o_x + o_dx and no_y > o_y and no_y + no_dy < o_y + o_dy:
                
                cv2.drawContours(frame, [no], 0, (0,255,0), 3)

    #cv2.imshow('Block Detection Test', thresh)
    cv2.imshow('hopefully a block', frame)
    cv2.imshow('Orange', orange_mask)
    cv2.imshow('white', white_mask)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

