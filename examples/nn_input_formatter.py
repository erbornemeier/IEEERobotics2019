import cv2
import numpy as np

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

photos = []
for letter in ['A','B','C','D','E','F']:
    for number in range(1,7):
        photo_index = "{}{}.jpg".format(letter,number)
        photos.append(photo_index)

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
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    block_contour = max(contours, key=cv2.contourArea)
    block_corners = find_rect_pts(block_contour).reshape((4,2)).tolist()
    block_corners = sorted(block_corners, key=lambda x: x[1])
    block_corners = sorted(block_corners[:2], key=lambda x: -x[0]),\
                    sorted(block_corners[2:], key=lambda x: x[0])
    block_corners = [*block_corners[0], *block_corners[1]]

    orig_pts = np.array(block_corners, dtype=np.float32)
    ortho_pts = np.array([[200,0],[0,0],[0,200],[200,200]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(orig_pts, ortho_pts)
    warp = cv2.warpPerspective(img, H, (200,200))

    thresh = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(thresh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    while(True):
        #cv2.imshow('original', img)
        cv2.imshow(image_index, thresh)
        #cv2.imshow('black_mask', black_mask)
        #cv2.imshow('mask',mask)
        #cv2.imshow('non-orange', non_orange_mask)
        #cv2.drawContours(img, [block_contour], 0, (0,255,0),3)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
