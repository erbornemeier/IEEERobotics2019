import numpy as np
import cv2

cap = cv2.VideoCapture(1)

lower_red = np.array([170,50,50])
upper_red = np.array([180,130,130])

lower_red2 = np.array([0,50,50])
upper_red2 = np.array([10,130,130])

lower_blue = np.array([110,150,0])
upper_blue = np.array([130,255,255])

lower_green = np.array([50,60,60])
upper_green = np.array([70,255,255])

lower_cyan = np.array([80,70,50])
upper_cyan = np.array([100,255,255])

lower_magenta = np.array([140,70,50])
upper_magenta = np.array([160,255,255])

lower_yellow = np.array([20,50,100])
upper_yellow = np.array([40,255,255])

lower_gray = np.array([0,0,80])
upper_gray = np.array([0,0,120])

while(True):
    try:
        colorVals = {}
        # Take each frame
        _, frame = cap.read()
        
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Convert to gray for circle detection
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grayFrame = cv2.medianBlur(grayFrame, 5)

        # get all of the circles in the image
        circles = cv2.HoughCircles(grayFrame, cv2.HOUGH_GRADIENT, 1, 500, param1 = 50, param2 = 40, minRadius = 50, maxRadius = 0)
        if circles is not None:
            
            # convert circles info to x,y,r 
            circles = np.round(circles[0]).astype("int")
            coin = circles[0]
            x = coin[0]
            y = coin[1]
            r = coin[2]
            cv2.circle(frame, (x,y), r ,(0,255,0),4)
            print("RADIUS: " + str(r))

            # create a cropped image to do color detection on
            cropped = hsv[y-r:y+r,x-r:x+r]

            #create masks for each color and count presence of each in 
            #cropped image
            red_mask = cv2.inRange(cropped, lower_red, upper_red)
            red_mask2 = cv2.inRange(cropped, lower_red2, upper_red2)
            num_red = cv2.countNonZero(red_mask)+ cv2.countNonZero(red_mask2)
            colorVals["Red"] = num_red

            blue_mask = cv2.inRange(cropped, lower_blue, upper_blue)
            num_blue = cv2.countNonZero(blue_mask)
            colorVals["Blue"] = num_blue

            green_mask = cv2.inRange(cropped, lower_green, upper_green)
            num_green = cv2.countNonZero(green_mask)
            colorVals["Green"] = num_green

            yellow_mask = cv2.inRange(cropped,lower_yellow, upper_yellow)
            num_yellow = cv2.countNonZero(yellow_mask)
            colorVals["Yellow"] = num_yellow

            cyan_mask = cv2.inRange(cropped, lower_cyan, upper_cyan)
            num_cyan = cv2.countNonZero(cyan_mask)
            colorVals["Cyan"] = num_cyan

            magenta_mask = cv2.inRange(cropped, lower_magenta, upper_magenta)
            num_magenta = cv2.countNonZero(magenta_mask)
            colorVals["Magenta"] = num_magenta

            gray_mask = cv2.inRange(cropped, lower_gray, upper_gray)
            num_gray = cv2.countNonZero(gray_mask)
            colorVals["Gray"] = num_gray

            # Bitwise-AND mask and original image
            #res = cv2.bitwise_and(frame,frame, mask= red_mask)

            #determine which color is most prevelent in the cropped image
            colorToPrint = max(colorVals, key=(lambda key: colorVals[key]))
            #if(colorVals[colorToPrint] < 4*pow(r,2)*.2):
            #    colorToPrint = "Gray"
            cv2.putText(frame, colorToPrint, (400,400), cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0))
            cv2.imshow("cropped",cropped)

        
        cv2.imshow('frame',frame)
        #cv2.imshow('mask',magenta_mask)
        #cv2.imshow('result',res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        pass
# When everything done, release the captue
cap.release()
cv2.destroyAllWindows()
