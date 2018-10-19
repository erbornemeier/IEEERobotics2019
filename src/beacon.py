import cv2
import numpy as np

BLUE_MASK = np.array([[ 80, 100, 110],
                      [150, 255, 255]])

def get_led_locations(frame, showContours=False, isRGBminArea=0.0, maxArea=float('inf')):
    #convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #threshold on color
    mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
    #close image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)) 
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    cv2.imshow('mask', mask)

    #find contours
    _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #filter by area
    #TODO

    if showContours:
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)


def main():

    cap = cv2.VideoCapture(0)

    while True:
        #get next frame
        ret, frame = cap.read()

        #unsuccessful frame capture
        if not ret:
            raise IOError("Frame could not be read")

        get_led_locations(frame, showContours=True)
        cv2.imshow('test', frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

if __name__ == '__main__':
    main()
