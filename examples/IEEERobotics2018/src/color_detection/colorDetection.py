import numpy as np
import cv2

cap = cv2.VideoCapture(0)

lower_red = np.array([150,160,80])
upper_red = np.array([180,255,255])

lower_blue = np.array([100,50,50])
upper_blue = np.array([140,255,255])

while(True):

    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    num_red = cv2.countNonZero(red_mask)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    num_blue = cv2.countNonZero(blue_mask)

    # Bitwise-AND mask and original image
    #res = cv2.bitwise_and(frame,frame, mask= red_mask)
    
    if num_red > num_blue:
        cv2.putText(frame,"RED", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255) )
    else:
        cv2.putText(frame,"BLUE", (100,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0))

    cv2.imshow('frame',frame)
    #cv2.imshow('mask',red_mask)
    #cv2.imshow('result',res)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()