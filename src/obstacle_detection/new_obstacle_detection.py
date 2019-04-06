import cv2
import numpy as np


cap = cv2.VideoCapture(1)

def get_obstacles(img, debug=False):
    orange_lower = np.array([0,60,0])                                              
    orange_upper = np.array([22,255,255])                                           
    white_lower = np.array([0,0,140])                                               
    white_upper = np.array([255,255,255])                                           
    kernel = np.ones((2,2),np.uint8)                                                
    kernel2 = np.ones((2,2),np.uint8) 
    kernel3 = np.ones((5,5),np.uint8) 

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                              
                                                                            
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)

    o, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    board = np.zeros(img.shape, dtype=np.uint8) 
    cv2.drawContours(board, [max(o, key=cv2.contourArea)], 0, (255,255,255), \
                                thickness=cv2.FILLED)
    board = cv2.bitwise_and(img, board)

    hsv = cv2.cvtColor(board, cv2.COLOR_BGR2HSV)                              
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    non_orange_mask = cv2.bitwise_not(orange_mask)
    
    def no_edge_touch(img, contour):
        h,w = img.shape[:2]
        x, y, ww, hh = cv2.boundingRect(contour)
        if  y <= 0:
            return False
        return True
    #cv2.imshow('nomask', non_orange_mask)
    contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_EXTERNAL, 
                                                      cv2.CHAIN_APPROX_SIMPLE) 
    contours = list(filter(lambda c: no_edge_touch(img, c), contours))
    for c in contours:
        if cv2.contourArea(c) > 4000:
            return True
    return False#cv2.drawContours(img,[c],0, (255,0,255), 3)

while True:
    _, img = cap.read()
    img = cv2.resize(img, (360,240))
    print(get_obstacles(img))
    #cv2.imshow('img', img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
        
