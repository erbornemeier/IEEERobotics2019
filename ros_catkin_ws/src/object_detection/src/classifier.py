#!/usr/bin/python

import cv2
import numpy as np
import os
import time as t
os.environ['TF_CPP_MIN_LOG_LEVEL']='3'

import tensorflow as tf
from keras.models import Sequential, load_model
from keras.backend import clear_session
clear_session()

import rospy
import rospkg
from sensor_msgs.msg import Image
from object_detection.srv import *
from cv_bridge import CvBridge

rospack = rospkg.RosPack()
bridge = CvBridge()

def find_rect_pts(pts):
    new_pts = pts
    epsilon = 0
    while len(new_pts) > 4:
        epsilon += 1
        new_pts = cv2.approxPolyDP(pts, epsilon, True)
    return new_pts

def load_h5_model():
    global letter_model
    path = rospack.get_path('object_detection') + '/src/letter_recognizer_new.h5'
    letter_model = load_model(path, compile=False)
    letter_model._make_predict_function()
    global graph
    graph = tf.get_default_graph()

def load_h5_slot_model():
    global slot_model
    path = rospack.get_path('object_detection') + '/src/slot_recognizer.h5'
    slot_model = load_model(path, compile=False)
    slot_model._make_predict_function()

load_h5_model()
load_h5_slot_model()
rospy.loginfo("loaded models!")

def image_recieved(data):
    global img
    img = bridge.imgmsg_to_cv2(data, "bgr8")

def classify_block(_):
    orange_lower = np.array([4,100,0])
    orange_upper = np.array([18,255,255])
    black_lower = np.array([0,0,0])
    black_upper = np.array([255,255,150])
    kernel = np.ones((5,5),np.uint8)
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
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        block_contour = max(contours, key=cv2.contourArea)
        if len(block_contour) < 4:
            raise Exception('Block face could not be found')
        #block_corners = find_rect_pts(block_contour).reshape((4,2)).tolist()
        rotated = cv2.minAreaRect(block_contour)
        block_corners = cv2.boxPoints(rotated)
        block_corners = sorted(block_corners, key=lambda x: x[1])
        block_corners = sorted(block_corners[:2], key=lambda x: -x[0]),\
                        sorted(block_corners[2:], key=lambda x: x[0])
        block_corners = [block_corners[0][0], block_corners[0][1], block_corners[1][0], block_corners[1][1]]

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

        with graph.as_default():
            guess = letter_model.predict(input_img).tolist()[0] 
            #if max(guess) > 0.95:
            guess = guess.index(max(guess))
            print("Guess: " + str( chr(guess + ord('A'))))
            return LetterResponse(guess) 
        
        #cv2.imshow("thresh", thresh)

    except Exception as e:
        print(e)
        return LetterResponse(0xFF)


def classify_slot(_):
    black_lower = np.array([0,0,0])
    black_upper = np.array([255,255,100])
    kernel = np.ones((4,4), np.uint8)
    padding = 7
    ideal_ratio = 0.6
    ratio_thresh = 0.4
    area_thresh = 50
    try:
        small_img = cv2.resize(img, (360, 240))
        h,w  = small_img.shape[:2]
        cx, cy = w//2, h//2
        hsv = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, black_lower, black_upper)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        _, blacks, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        middle_black = None
        min_dist = float('inf')
        for b in blacks:
            if cv2.contourArea(b) > area_thresh:
                M = cv2.moments(b)
                if M['m00'] != 0:
                    x = M['m10']/M['m00']
                    y = M['m01']/M['m00']
                    dist = (cx-x)**2 + (cy-y)**2
                    if dist < min_dist:
                        bx, by, bw, bh = cv2.boundingRect(b)
                        if abs(bw/float(bh) - ideal_ratio) < ratio_thresh:
                            min_dist = dist
                            middle_black = b
        if middle_black is not None:
            bx, by, bw, bh = cv2.boundingRect(middle_black)
            out = thresh[by-padding:by+bh+padding, bx-padding:bx+bw+padding]
            out = cv2.resize(out, (40,60))
        else:
            raise Exception('could not find letter')

        input_img = out.astype('float32')
        input_img /= 255.
        input_img = input_img.reshape(1, 40*60)

        with graph.as_default():
            guess = slot_model.predict(input_img).tolist()[0]                            
            confidence = max(guess)
            print("CONFIDENCE {}".format(confidence))
            guess = guess.index(max(guess))
            print("Guess: " + str( guess))
            return LetterResponse(guess)
    except Exception as e:
        print(e)
        return LetterResponse(0xFF)


rospy.init_node("letter_classify", anonymous=True)
rospy.Subscriber("camera_image", Image, image_recieved)
letter_srv = rospy.Service("letter_identifier", Letter, classify_block )
slot_srv = rospy.Service("slot_identifier", Letter, classify_slot)
rospy.spin()

