import pickle
import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

from keras.datasets import mnist
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout, Activation
from keras.utils import np_utils

model = load_model("results/letter_recognizer.h5")

with open('outputs.pickle', 'rb') as handle:
    nn_inputs = pickle.load(handle) 

for img in nn_inputs.values():
    input_img = img.flatten()
    input_img = input_img.astype('float32')
    input_img /= 255.
    input_img = input_img.reshape(1,784)

    guess = model.predict(input_img).tolist()[0] 
    print("Guess: " + str(guess))
    while True:
        #cv2.imshow("img", img)
        cv2.imshow("img", cv2.resize(img, (500,500)))
        
        key = cv2.waitKey(1)
        if key == ord('n'):
            break
