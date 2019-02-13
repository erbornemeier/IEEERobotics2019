import cv2
import pickle
import numpy as np
import random

num_noises = 200

def find_rect_pts(pts):
    new_pts = pts
    epsilon = 0
    while len(new_pts) > 4:
        epsilon += 1
        new_pts = cv2.approxPolyDP(pts, epsilon, True)
    return new_pts

def sp_noise(image,prob):
    '''
    Add salt and pepper noise to image
    prob: Probability of the noise
    '''
    output = np.zeros(image.shape,np.uint8)
    thres = 1 - prob 
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            rdn = random.random()
            if rdn < prob:
                output[i][j] = 0
            elif rdn > thres:
                output[i][j] = 255
            else:
                output[i][j] = image[i][j]
    return output

orange_lower = np.array([4,100,0])
orange_upper = np.array([18,255,255])
black_lower = np.array([0,0,0])
black_upper = np.array([255,255,150])
kernel = np.ones((5,5),np.uint8)

photos = []
for letter in ['A','B','C','D','E','F']:
    for number in range(1,7):
        photo_index = "{}{}.jpg".format(letter,number)
        photos.append(photo_index)

outputs = dict()

for image_index in photos:
    img = cv2.imread('nnInputPictures/'+image_index)
    img = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
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

    rows, cols = thresh.shape
    up = thresh
    up = up[1:101,1:101]
    #adding noise
    M = cv2.getRotationMatrix2D((cols/2.0,rows/2.0),90,1)
    left = cv2.warpAffine(thresh, M, (cols,rows))
    down = cv2.warpAffine(left, M, (cols,rows))
    right = cv2.warpAffine(down, M, (cols,rows))

    for i in range(num_noises):
        up_n = sp_noise(up,0.05)
        left_n = sp_noise(left,0.05)
        down_n = sp_noise(down,0.05)
        right_n = sp_noise(right,0.05)

        outputs[image_index+"U"+str(i)] = cv2.resize(up_n, (28,28))
        outputs[image_index+"L"+str(i)] = cv2.resize(left_n, (28,28)) 
        outputs[image_index+"R"+str(i)] = cv2.resize(right_n, (28,28)) 
        outputs[image_index+"D"+str(i)] = cv2.resize(down_n, (28,28)) 

    print ("Finished: " + image_index)
    '''
    while(True):
        #cv2.imshow('original', img)
        #cv2.imshow(image_index, up)
        #cv2.imshow("left", left)
        #cv2.imshow("down", down)
        #cv2.imshow("right", right)
        #cv2.imshow('black_mask', black_mask)
        #cv2.imshow('mask',mask)
        #cv2.imshow('non-orange', non_orange_mask)
        #cv2.drawContours(img, [block_contour], 0, (0,255,0),3)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
    '''

with open('outputs.pickle', 'wb') as handle:
    pickle.dump(outputs, handle, protocol=pickle.HIGHEST_PROTOCOL)

