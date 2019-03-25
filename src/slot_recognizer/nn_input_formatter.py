#!/usr/bin/python3
import cv2
import numpy as np
import math
import pickle

black_lower = np.array([0,0,0])
black_upper = np.array([255,255,80])
kernel = np.ones((4,4),np.uint8)

photos = []
for i in range(1,61):
    photo_index = "ABC_{}.jpg".format(str(i).rjust(2, '0'))
    photos.append(photo_index)
    photo_index = "DEF_{}.jpg".format(str(i).rjust(2, '0'))
    photos.append(photo_index)

outputs = dict()

for image_index in photos:
    img = cv2.imread('images/' + image_index)
    img = cv2.resize(img, (360,240))
    h,w = img.shape[:2]
    cx = w//2
    cy = h//2
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #_, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    thresh = cv2.inRange(hsv, black_lower, black_upper)
    #_, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU )
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    blacks, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    if len(blacks) != 0:
        middle_black = blacks[0]
        middle_black_dist = 100000

        for b in blacks:
            M = cv2.moments(b)
            if M['m00'] != 0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
                dist = math.sqrt(pow(cx-x,2)+pow(cy-y,2))
                if  dist < middle_black_dist:
                    middle_black_dist = dist
                    middle_black = b

        bx,by,bw,bh = cv2.boundingRect(middle_black)

        cv2.rectangle(img,(bx-7,by-7),(bx+bw+7,by+bh+7),(0,255,0),2)
        out = thresh[by-7:by+bh+7,bx-7:bx+bw+7]
        out = cv2.resize(out, (40,60))
        print(image_index)
        '''
        while(True):
            cv2.imshow("black", out)
            cv2.imshow("thresh", thresh)
            cv2.imshow("og", img) 
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
        '''
    outputs[image_index] = out

with open('outputs.pickle', 'wb') as handle:
    pickle.dump(outputs, handle, protocol=pickle.HIGHEST_PROTOCOL)

