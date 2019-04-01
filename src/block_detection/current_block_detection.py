import cv2
import numpy as np

orange_lower = np.array([4,50,100])                                              
orange_upper = np.array([18,255,255])                                           
white_lower = np.array([0,0,140])                                               
white_upper = np.array([255,255,255])                                           
kernel = np.ones((5,5),np.uint8)                                                
kernel2 = np.ones((5,5),np.uint8) 
kernel3 = np.ones((10,10),np.uint8) 

cap = cv2.VideoCapture(1)

diff_weight = 1
aspect_weight = 20
min_goodness = 35

def average_contrast(img, contour, debug=False):
    x, y, w, h = cv2.boundingRect(contour)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    black = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.drawContours(black, [contour], 0, (255,255,255), thickness=cv2.FILLED)
    cropped = cv2.bitwise_and(gray, black)
    if cropped.size == 0:
        return 0
    avg_pixel_value = np.sum(cropped) / np.count_nonzero(cropped)

    diff = cv2.bitwise_and(black.astype(np.float64), np.abs(cropped - avg_pixel_value))
    pixel_diffs = np.sum(diff)
    avg_pixel_diff = pixel_diffs / np.count_nonzero(diff)
    aspect_error = max(w/float(h), h/float(w))-1
    goodness =  avg_pixel_diff*diff_weight - aspect_error*aspect_weight 
    if debug:
        cv2.putText(img, str(round(goodness, 3)), (x+w//2, y+h//2),\
                 cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255), 2)
    return goodness

def get_block_contour(img, debug=False):
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
            return max_contrast[0]

    non_orange_mask = cv2.bitwise_not(orange_mask)
    _, contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_EXTERNAL, 
                                                  cv2.CHAIN_APPROX_SIMPLE) 
    goodness_values = [(c, average_contrast(img, c, debug=debug)) for c in contours]
    max_contrast = max(goodness_values, key=lambda x: x[1])
    if debug:
        cv2.imshow('no_mask', non_orange_mask)
    if max_contrast[1] > min_goodness:
        return max_contrast[0]
    return False
         

while True:
    _, img = cap.read()
    block = get_block_contour(img, debug=True)
    if block is not False:
        cv2.drawContours(img, [block], 0, (0,255,0), 3)
        
    cv2.imshow('img', img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
        
