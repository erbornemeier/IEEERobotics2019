import numpy as np
import cv2

def nothing(x):
    pass

cap = cv2.VideoCapture(1)

lower_red = np.array([150,160,80])
upper_red = np.array([180,255,255])

lower_blue = np.array([100,50,50])
upper_blue = np.array([140,255,255])

cv2.namedWindow('lineDetect')

cv2.createTrackbar('rho', 'lineDetect', 1, 255, nothing)
cv2.createTrackbar('bin_thresh', 'lineDetect', 1, 255, nothing)
cv2.createTrackbar('thresh', 'lineDetect', 1, 1000, nothing)

while(True):

    rho = cv2.getTrackbarPos('rho', 'lineDetect')
    bin_thresh = cv2.getTrackbarPos('bin_thresh', 'lineDetect')
    thresh = cv2.getTrackbarPos('thresh', 'lineDetect')
    print("r: {}, bin_thresh: {}, thresh: {}".format(rho, bin_thresh, thresh))

    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bins = cv2.threshold(gray, bin_thresh, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(bins, 50, 150, apertureSize = 3)

    #lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
    lines = cv2.HoughLinesP(edges, rho, np.pi/180, thresh)
    if lines is not None: 
        for line in lines:
            print(line)
            '''
            for rho,theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
            '''
            for x1, y1, x2, y2 in line:
                cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
                

    '''
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
    '''

    cv2.imshow('lineDetect',frame)
    cv2.imshow('canny', edges)
    cv2.imshow('bins', bins)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
