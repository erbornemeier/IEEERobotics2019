import cv2
import numpy as np

black_lower = np.array([0,0,0])                                               
black_upper = np.array([255,255,80])                                           
white_lower = np.array([0, 0, 210])
white_upper = np.array([255,255,255])                                           
orange_lower = np.array([4, 50, 100])
orange_upper = np.array([18, 255, 255])
kernel = np.ones((7,7),np.uint8)                                                
kernel2 = np.ones((17,17),np.uint8) 

cap = cv2.VideoCapture(1)

while True:
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                              
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                              
                                                                            
    black_mask = cv2.inRange(hsv, black_lower, black_upper)
    white_mask = cv2.inRange(hsv, white_lower, white_upper)
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)

    _, contours, hier = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maxIndex = np.argmax([cv2.contourArea(c) for c in contours])
    interiors = []
    AREA_THRESH = 300
    for i, h in enumerate(hier[0]):
        if h[3] == maxIndex:
            if cv2.contourArea(contours[i]) > AREA_THRESH:
                interiors.append(contours[i])
    masks = []
    for i in interiors:
        mask = np.ones(img.shape[:2], dtype="uint8")*255
        cv2.drawContours(mask, [i], 0, 0, -1)
        mask = cv2.bitwise_not(mask)
        result = cv2.bitwise_and(gray, gray, mask=mask)
        _, result = cv2.threshold(result, 35, 255, cv2.THRESH_BINARY)
        masks.append(result)
    global_mask = masks[0]
    for mask in masks[1:]:
        global_mask = cv2.bitwise_or(global_mask, mask)
    final = cv2.bitwise_and(gray, gray, mask=global_mask)
        

    #cv2.drawContours(img, interiors, -1, (0,255,0), 3)
    cv2.imshow('interiors', global_mask) 

    #non_orange_mask = cv2.bitwise_not(orange_mask)
    #non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_OPEN, kernel)
    #non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_CLOSE, kernel)

    #white_mask = cv2.inRange(hsv, white_lower, white_upper)
    #white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
    #white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.bitwise_and(white_mask, non_orange_mask)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel2)

    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 9, 2.25)
    #thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    #thresh = cv2.dilate(thresh, kernel2)

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

    #cv2.imshow('thresh', thresh)
    #cv2.imshow('white', white_mask)
    #cv2.imshow('black', black_mask)
    #cv2.imshow('orange', orange_mask)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
        
