import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

from keras.models import Sequential, load_model

model = load_model("results/slot_recognizer2.h5")

#cap = cv2.VideoCapture(1)

with open('outputs.pickle', 'rb') as handle:
    nn_inputs = pickle.load(handle) 

for label, img in nn_inputs:
    #_, img = cap.read()
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray,(100,100))

        #input_img = gray.flatten()
        input_img = gray
        input_img = input_img.astype('float32')
        input_img /= 255.
        input_img = input_img.reshape(-1,100,100,1)

        guess = model.predict(input_img).tolist()[0] 
        print(guess)
        guess = guess.index(max(guess))
        print("Guess: " + ("ABC" if guess==1 else "DEF"))

        cv2.imshow("img", img)
    except Exception as e:
        print(e)
        pass
        
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
