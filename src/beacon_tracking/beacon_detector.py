import cv2
import numpy as np
from comm import i2c_comm 

BLUE_MASK = np.array([[ 85, 190, 158],
                      [130, 255, 255]])

LINE_THRESHOLD = 100 
PIXEL2INCH = 1.0 / 262.0 

def get_led_locations(frame, showContours=False, isRGBminArea=0.0, maxArea=float('inf')):
    #convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #threshold on color
    mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
    #open image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8)) 
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel)

    cv2.imshow('mask', mask)

    #find contours
    _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #filter by area
    #TODO
    if contours:
        centers = []

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
                centers.append((cx, cy))

                if showContours:
                    cv2.circle(frame, (int(cx), int(cy)), 7, (0,0,255), -1)

        top, middle, bottom = None, None, None
        smallestDist = 10000000

        for i, c_top in enumerate(centers):
            for j, c_bottom in enumerate(centers):
                if i != j:
                    midpt = ((c_top[0] + c_bottom[0]) // 2, (c_top[1] + c_bottom[1]) // 2)
                    for k, c_middle in enumerate(centers):
                        if k != i and k != j:
                            dist = (c_middle[0] - midpt[0])**2 + (c_middle[1] - midpt[1])**2
                            if dist < smallestDist:
                                smallestDist = dist
                                top, middle, bottom = c_top, c_middle, c_bottom 

       
        if top is not None and smallestDist < LINE_THRESHOLD:
            cv2.line(frame, (int(top[0]), int(top[1])), (int(bottom[0]), int(bottom[1])), (255,255,0), 5)
            pixels =  ((top[0] - bottom[0]) ** 2 + (top[1] - bottom[1])**2) ** 0.5
            #print(pixels)
            inches = 12 / (pixels * PIXEL2INCH)
            print(inches) 
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)




def main():

    cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0 )	

    communication = False
    try:
        comm = i2c_comm(0x28)
        communication = True
    except Exception:
        print('COMMUNICATION COULD NOT BE ESTABLISHED')

    width, height = cap.get(3), cap.get(4)
    print(width)

    while True:
        #get next frame
        for i in range(0,1):
            ret, frame = cap.read()

        #unsuccessful frame capture
        if not ret:
            raise IOError("Frame could not be read")

        center = get_led_locations(frame, showContours=True)
        if center:
                offset_from_center = -int(center[1] - height/2)
                print(offset_from_center)
                if communication:
                    comm.sendLEDPos([offset_from_center])
        cv2.imshow('test', frame)

        key = cv2.waitKey(100)
        if key == ord('q'):
            break

if __name__ == '__main__':
    main()
