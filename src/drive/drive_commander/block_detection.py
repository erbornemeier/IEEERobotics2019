import cv2
import numpy as np

class block_detector:
    def __init__(self):
        self.board_lower = np.array([4,100,9])
        self.board_upper = np.array([18,255,255])
        self.kernel = np.ones((7,7), np.uint8)

    def is_contained(self, bb1, bb2):
        a_x, a_y, a_w, a_h = bb1
        b_x, b_y, b_w, b_h = bb2
        return  a_x > b_x\
            and a_y > b_y\
            and a_x + a_w < b_x + b_w\
            and a_y + a_h < b_y + b_h\
            and a_w > 50 and a_h > 50

    def get_block_contours(self, frame):

        #convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #get the orange contours in the image
        orange_mask = cv2.inRange(hsv, self.board_lower, self.board_upper)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, self.kernel)
        _, orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #get the non orange contours in the image
        non_orange_mask = cv2.bitwise_not(orange_mask)
        _, non_orange_contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #find nonorange contours within orange ones
        block_contours = []
        for no in non_orange_contours:
            no_bb = cv2.boundingRect(no)
            for o in orange_contours:
                o_bb = cv2.boundingRect(o)
                if self.is_contained(no_bb, o_bb):
                    block_contours.append(no)
        return block_contours

'''
cap = cv2.VideoCapture(1)

orange_lower = np.array([4,100,9])
orange_upper = np.array([18,255,255])
kernel = np.ones((7,7),np.uint8)

while True:
    _, frame = cap.read()

    #threshold the image
    thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.GaussianBlur(thresh, (5,5), 0)
    thresh = cv2.adaptiveThreshold(thresh, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #get the orange contours in the image
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #get the non orange contours in the image
    non_orange_mask = cv2.bitwise_not(orange_mask)
    non_orange_contours, _ = cv2.findContours(non_orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #get the white contours in the image
    white_contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #invert and get the black contours in the image
    thresh2 = cv2.bitwise_not(thresh)
    black_contours, _ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    #find nonorange contours within orange ones
    for no in non_orange_contours:
        no_x, no_y, no_dx, no_dy = cv2.boundingRect(no)
        for o in orange_contours:
            o_x, o_y, o_dx, o_dy = cv2.boundingRect(o)
            if no_dx > 50 and no_dy > 50 and  no_x > o_x and no_x + no_dx < o_x + o_dx and no_y > o_y and no_y + no_dy < o_y + o_dy:
                
                cv2.drawContours(frame, [no], 0, (0,255,0), 3)

    #cv2.imshow('Block Detection Test', thresh)
    cv2.imshow('hopefully a block', frame)
    cv2.imshow('Orange', orange_mask)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

'''
