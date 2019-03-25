import cv2
import math
import numpy as np
import os
import pickle
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

from keras.models import Sequential, load_model

model = load_model("results/slot_recognizer.h5")

cap = cv2.VideoCapture(1)

with open('outputs.pickle', 'rb') as handle:
    nn_inputs = pickle.load(handle) 

black_lower = np.array([0,0,0])
black_upper = np.array([255,255,80])
kernel = np.ones((4,4),np.uint8)

while True:
    _, img = cap.read()
    try:
        #preprocess
        img = cv2.resize(img, (360,240))
        h,w = img.shape[:2]
        cx = w//2
        cy = h//2
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, black_lower, black_upper)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        blacks, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        if len(blacks) != 0:
            middle_black = blacks[0]
            middle_black_dist = float('inf') 

            for b in blacks:
                M = cv2.moments(b)
                if M['m00'] != 0:
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])
                    dist = math.sqrt(pow(cx-x,2)+pow(cy-y,2))
                    if  dist < middle_black_dist:
                        middle_black_dist = dist
                        middle_black = b

            bx,by,bw,bh = cv2.boundingRect(middle_black)

            cv2.rectangle(img,(bx-7,by-7),(bx+bw+7,by+bh+7),(0,255,0),2)
            out = thresh[by-7:by+bh+7,bx-7:bx+bw+7]
            out = cv2.resize(out, (40,60))
 
        input_img = out 

        #classify
        input_img = input_img.astype('float32')
        input_img /= 255.
        input_img = input_img.reshape(1, 40*60)

        guess = model.predict(input_img).tolist()[0] 
        print(guess)
        guess = guess.index(max(guess))
        print("Guess: " + ("DEF" if guess==1 else "ABC"))

        cv2.imshow("img", img)
    except Exception as e:
        print(e)
        pass
        
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
