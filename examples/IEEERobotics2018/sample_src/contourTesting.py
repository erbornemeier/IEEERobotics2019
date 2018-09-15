import cv2
import numpy as np

cap = cv2.VideoCapture(1);
while True:
    #capture a convert to grayscale
    _, img = cap.read()
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #convert to binary
    ret, thresh = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    cnt = 0
    centroid = (0,0)
    if 0 < len(contours) < 3:
        area, cnt = max([(cv2.contourArea(c), c) for c in contours])
        M = cv2.moments(cnt)
        centroid = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
        if area > 100000:
            circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 100, 
                                       param1=50, param2=30, minRadius=20,
                                       maxRadius=150)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    cv2.circle(img, (i[0], i[1]), i[2], (0, 0, 255), 2)

    cv2.drawContours(img, contours, -1, (0,255,0), 3)
    cv2.circle(img, centroid, 5, (0, 0, 255))
    cv2.putText(img,"Area: {}".format(area), (0, 470), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 3)
    cv2.imshow('dst',img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break
        #https://docs.opencv.org/3.3.1/d7/d4d/tutorial_py_thresholding.html
