import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
template = cv.imread('pi_box.jpg',0)

w, h = template.shape[::-1]

while True:
    _,img = cap.read(); 
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Apply template Matching
    res = cv.matchTemplate(img,template,cv.TM_CCOEFF)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv.rectangle(img,top_left, bottom_right, 255, 2)
    if cv.waitKey(5) & 0xFF == 27:
        break
    cv.imshow("Template Matcher",img) 
