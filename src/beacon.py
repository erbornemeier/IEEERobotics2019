import cv2
import numpy as np
from comm import i2c_comm 

BLUE_MASK = np.array([[100, 160, 205],
                      [179, 255, 255]])

def get_led_locations(frame, showContours=False, isRGBminArea=0.0, maxArea=float('inf')):
    #convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #threshold on color
    mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
    #open image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)) 
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    cv2.imshow('mask', mask)

    #find contours
    _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #filter by area
    #TODO
    if contours:
        #get biggest contour
        biggest = max(contours, key=cv2.contourArea)

        M = cv2.moments(biggest)
        cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

        if showContours:
            cv2.drawContours(frame, np.array([biggest]), -1, (0,255,0), 3)
            cv2.circle(frame, (cx, cy), 7, (0,0,255), -1)

        return (cx, cy)


def main():

    cap = cv2.VideoCapture(0)
    comm = i2c_comm(0x28)
    width, height = cap.get(3), cap.get(4)

    while True:
        #get next frame
        ret, frame = cap.read()

        #unsuccessful frame capture
        if not ret:
            raise IOError("Frame could not be read")

        center = get_led_locations(frame, showContours=True)
        offset_from_center = width/2 - center[0]
        comm.sendLEDPos(offset_from_center)
        cv2.imshow('test', frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

if __name__ == '__main__':
    main()
