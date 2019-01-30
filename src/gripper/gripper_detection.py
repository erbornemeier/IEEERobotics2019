import cv2
import numpy as np


cyan_lower = np.array([89,130,98])
cyan_upper = np.array([115,255,255])
kernel = np.ones((10,10), np.uint8)
blockX = 250
blockY = 250
threshold = 25

def checkPinch(c1X, c1Y, c2X, c2Y):
    if abs(c1Y - blockY) < threshold and abs(c2Y - blockY) < threshold and \
        c1X < blockX and c2X > blockX:
        return True
    else:
        return False


cap = cv2.VideoCapture(0)

while True:
    
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cyan_mask = cv2.inRange(hsv, cyan_lower, cyan_upper)
    cyan_mask = cv2.morphologyEx(cyan_mask, cv2.MORPH_CLOSE, kernel)
    cyan_mask = cv2.morphologyEx(cyan_mask, cv2.MORPH_OPEN, kernel)

    cyan_contours, _ = cv2.findContours(cyan_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(cyan_contours) > 1:
        cyan_contours = sorted(cyan_contours, key=cv2.contourArea)
        c1 = cyan_contours[0]
        c2 = cyan_contours[1]

        M1 = cv2.moments(c1)
        M2 = cv2.moments(c2)
        c1X = int(M1["m10"] / M1["m00"])
        c1Y = int(M1["m01"] / M1["m00"])
        c2X = int(M2["m10"] / M2["m00"])
        c2Y = int(M2["m01"] / M2["m00"])

        cv2.circle(frame, (c1X, c1Y), 7, (255, 255, 255), -1)
        cv2.circle(frame, (c2X, c2Y), 7, (255,255,255), -1)

        cv2.putText(frame, "Dots!", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))

        if c2X < c1X:
            tmpX, tmpY = c1X, c1Y
            c1X, c1Y = c2X, c2Y
            c2X, c2Y = tmpX, tmpY

        if(checkPinch(c1X,c1Y,c2X,c2Y)):
            cv2.putText(frame, "Pinch!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
    else:
        cv2.putText(frame, "Can't see dots", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
    
    cv2.circle(frame, (blockX,blockY), 7, (255,0,255), -1)
    cv2.imshow('dots', frame)
    cv2.imshow('mask', cyan_mask)

    key = cv2.waitKey(1)
    if key == ord('q'):
        cv2.destroyAllWindows()
        break
