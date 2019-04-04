import cv2
import numpy as np


cap = cv2.VideoCapture(1)

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
    
    cv2.imshow('mask', mask)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                                      cv2.CHAIN_APPROX_SIMPLE) 
    AREA_THRESH = 500
    contours = list(filter(lambda c: cv2.contourArea(c) > AREA_THRESH, contours))
    isolated = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.drawContours(isolated, contours, -1, (255,255,255), thickness=cv2.FILLED)
    _, diff = cv2.threshold(np.absolute(isolated-mask), 128, 255, cv2.THRESH_BINARY)
    #diff = cv2.morphologyEx(diff, cv2.MORPH_CLOSE, kernel3)
    diff = cv2.dilate(diff, kernel3)
    cv2.imshow('iso', diff)

    _, letters, _ = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    def no_edge_touch(img, contour):
        h,w = img.shape[:2]
        x, y, ww, hh = cv2.boundingRect(contour)
        if x <= 0 or y <= 0 or ww >= w or hh >= h:
            return False
        return True
    letters = list(filter(lambda c: no_edge_touch(img, c), letters))
    print(len(letters))

    if len(letters) > 0:
        letter = max(letters, key=cv2.contourArea)
        return letter 
    return False

while True:
    _, img = cap.read()
    img = cv2.resize(img, (360,240))
    block = get_block_contour(img, debug=True)
    if block is not False:
        cv2.drawContours(img, [block], 0, (0,255,0), 2)
    cv2.imshow('img', img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
        
