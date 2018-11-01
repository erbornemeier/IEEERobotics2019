import cv2
import numpy as np


class beacon_tracker:
    
    LED_MASK = np.array([[ 85, 190, 158],
                          [130, 255, 255]])
    LINE_THRESHOLD = 100 
    PIXEL2INCH = 1.0 / (262.0*12) 
    
    def get_LED_locations(self, frame, show=False):
        #get potential LED locations
        contours = self.__find_potential_LEDs__(frame, show=show)
        if not contours:
            return False
       
        #find the center of all potential LEDs
        centers = []
        for c in contours:
           center = self.__get_center_of_contour__(c, show=show) 
           if center:
               centers.append(center)

        #find the three LEDs that best form a straight line
        leds = self.__find_LED_line__(centers, show=show)
        if not leds:
            return False

        #return in order top, middle, bottom
        return leds


    def estimate_distance_to_beacon(self, leds):
        top, _, bottom = leds
        #if the top and bottom were found
        if top is not None and bottom is not None:
            #get distance between them in pixels
            pixels =  ((top[0] - bottom[0]) ** 2 + (top[1] - bottom[1])**2) ** 0.5
            #convert to inches
            if pixels > 4:
                inches = pixels * self.PIXEL2INCH
                return inches
        return False


    def __find_potential_LEDs__(self, frame, show=False):
        #convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #threshold on color
        mask = cv2.inRange(hsv, self.LED_MASK[0], self.LED_MASK[1])
        #dilate image to expand contours
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12,12)) 
        mask = cv2.dilate(mask, kernel)
        #find contours
        _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #show mask and contours
        if show:
            cv2.imshow('Potential LEDS', mask)
            cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        return contours 


    def __get_center_of_contour__(self, contour, show=False):
        # get moments of contour
        M = cv2.moments(contour)
        #if large enough to have center
        if M['m00'] > 0:
            #get center point
            cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
            if show:
                cv2.circle(frame, (int(cx), int(cy)), 7, (0,0,255), -1)
            return (cx, cy)
        return False


    def __find_LED_line__(self, centers, show=False):
        top, middle, bottom = None, None, None 
        smallestDiff = float('inf')
        for c_top in centers:
            for c_bottom in centers:
                if c_top != c_bottom:
                    midpt = ((c_top[0] + c_bottom[0]) // 2, (c_top[1] + c_bottom[1]) // 2)
                    for c_middle in centers:
                        if c_middle not in [c_top, c_bottom]:
                            middle_diff = (c_middle[0] - midpt[0])**2 + (c_middle[1] - midpt[1])**2
                            if middle_diff < smallestDiff:
                                smallestDiff = middle_diff
                                top, middle, bottom = c_top, c_middle, c_bottom 
        if smallestDiff < self.LINE_THRESHOLD:
            if show:
                cv2.line(frame, (int(top[0]), int(top[1])), (int(bottom[0]), int(bottom[1])), (255,255,0), 5)
            return (top, middle, bottom)
        return False



#TEST OF BEACON TRACKER
if __name__ == '__main__':

    cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0 )	

    tracker = beacon_tracker(servoID=0)

    width, height = cap.get(3), cap.get(4)

    while True:
        #get next frame
        for i in range(0,6):
            ret, frame = cap.read()

        #unsuccessful frame capture
        if not ret:
            raise IOError("Frame could not be read")

        leds = tracker.get_LED_locations(frame, show=True)
        if leds:
            dist_to_beacon = tracker.estimate_distance_to_beacon(leds)
            middle = leds[1]
            dist_to_middle = (width/2 - middle[0])
            if dist_to_beacon:
                print("To beacon (in): {}\tTo middle (px): {}".format(dist_to_beacon, dist_to_middle))
        cv2.imshow('Beacon Tracker Test', frame)

        key = cv2.waitKey(100)
        if key == ord('q'):
            break

