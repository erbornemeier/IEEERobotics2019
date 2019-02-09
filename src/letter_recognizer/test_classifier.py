import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

from keras.models import Sequential, load_model

def find_rect_pts(pts):
    new_pts = pts
    epsilon = 0
    while len(new_pts) > 4:
        epsilon += 1
        new_pts = cv2.approxPolyDP(pts, epsilon, True)
    return new_pts

orange_lower = np.array([4,100,0])
orange_upper = np.array([18,255,255])
black_lower = np.array([0,0,0])
black_upper = np.array([255,255,150])
kernel = np.ones((5,5),np.uint8)

model = load_model("results/letter_recognizer.h5")

cap = cv2.VideoCapture(1)

while True:
    _, img = cap.read()
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        black_mask = cv2.inRange(hsv, black_lower, black_upper)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.bitwise_not(black_mask)

        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
        non_orange_mask = cv2.bitwise_not(orange_mask)
        non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_CLOSE, kernel)
        non_orange_mask = cv2.morphologyEx(non_orange_mask, cv2.MORPH_OPEN, kernel)
        
        mask = cv2.bitwise_and(non_orange_mask, white_mask)
        if cv2.__version__ == '3.4.0':
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        block_contour = max(contours, key=cv2.contourArea)
        block_corners = find_rect_pts(block_contour).reshape((4,2)).tolist()
        block_corners = sorted(block_corners, key=lambda x: x[1])
        block_corners = sorted(block_corners[:2], key=lambda x: -x[0]),\
                        sorted(block_corners[2:], key=lambda x: x[0])
        block_corners = [*block_corners[0], *block_corners[1]]

        orig_pts = np.array(block_corners, dtype=np.float32)
        ortho_pts = np.array([[102,0],[0,0],[0,102],[102,102]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(orig_pts, ortho_pts)
        warp = cv2.warpPerspective(img, H, (102,102))

        thresh = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(thresh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        thresh_small = cv2.resize(thresh, (28,28))

        input_img = thresh_small.flatten()
        input_img = input_img.astype('float32')
        input_img /= 255.
        input_img = input_img.reshape(1,784)

        guess = model.predict(input_img).tolist()[0] 
        guess = guess.index(max(guess))
        print("Guess: " + str(chr(guess + ord('A'))))

        cv2.imshow("thresh", thresh)
        cv2.imshow("img", img)
    except Exception:
        pass
        
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
