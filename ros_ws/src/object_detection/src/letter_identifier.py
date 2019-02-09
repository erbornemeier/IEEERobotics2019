#!/usr/bin/python

import cv2
import numpy as np

from keras.models import load_model

import rospy

orange_lower = np.array([4,100,0])
orange_upper = np.array([18,255,255])
black_lower = np.array([0,0,0])
black_upper = np.array([255,255,150])
kernel = np.ones((5,5),np.uint8)
model = load_model("letter_recognizer_model.h5")
nn_input_size = 28
cap = cv2.VideoCapture(0)


def find_rect_pts(pts):
    new_pts = pts
    epsilon = 0
    while len(new_pts) > 4:
        epsilon += 1
        new_pts = cv2.approxPolyDP(pts, epsilon, True)
    return new_pts


def threshold_image(img):
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
    return mask


def get_tf_to_block(mask):
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    block_contour = max(contours, key=cv2.contourArea)
    block_corners = find_rect_pts(block_contour).reshape((4,2)).tolist()
    block_corners = sorted(block_corners, key=lambda x: x[1])
    block_corners = sorted(block_corners[:2], key=lambda x: -x[0]),\
                    sorted(block_corners[2:], key=lambda x: x[0])
    block_corners = [block_corners[0][0], block_corners[0][1],\
                     block_corners[1][0], block_corners[1][1]

    orig_pts = np.array(block_corners, dtype=np.float32)
    ortho_pts = np.array([[nn_input_size,0],\
                          [0,0],\
                          [0,nn_input_size],\
                          [nn_input_size,nn_input_size]],\
                          dtype=np.float32)
    H = cv2.getPerspectiveTransform(orig_pts, ortho_pts)
    return H


def classify_block(img):
    input_img = img.flatten()
    input_img = input_img.astype('float32')
    input_img /= 255.
    input_img = input_img.reshape(1,784)

    guess = model.predict(input_img).tolist()[0] 
    guess = guess.index(max(guess))
    return chr(guess + ord('A'))

rospy.init_node('block_identifier', anonymous=True)
rate = rospy.Rate(10)
rospy.loginfo("started letter identifier")

while rospy.is_shutdown():
    try:
        _, img = cap.read()
        mask = threshold_image(img)
        tf   = get_tf_to_block(mask)
        warp = cv2.warpPerspective(img, tf, (nn_input_size,nn_input_size))
        thresh = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(thresh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        letter = classify_block(thresh)
        rospy.loginfo(letter)
        rate.sleep()
    except Exception as e:
        rospy.logerror(e)
        pass
