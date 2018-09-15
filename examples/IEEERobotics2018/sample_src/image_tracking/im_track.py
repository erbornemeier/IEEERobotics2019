
import cv2 
import numpy as np
import math

cap = cv2.VideoCapture(0)
REFERENCE_IMG = "pi_box.jpg"
MIN_MATCH_CT = 10 
FLANN_INDEX_KDTREE = 0

sift = cv2.xfeatures2d.SIFT_create()
ref_img = cv2.imread(REFERENCE_IMG, 0) 
ref_kp, ref_des = sift.detectAndCompute(ref_img, None)

index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

read = True
while True:
    
    if read:
        read = False
        _, frame = cap.read()
    else:
        read = True
        continue
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    target_kp, target_des = sift.detectAndCompute(grey, None)
    matches = flann.knnMatch(np.asarray(ref_des, np.float32), np.asarray(target_des, np.float32),k=2)

    good = []
    for m,n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_CT:
        src_pts = np.float32([ ref_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ target_kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h,w = ref_img.shape
        pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
        if M is not None:
            dst = cv2.perspectiveTransform(pts, M)
            frame = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

            sy = math.sqrt(M[0,0] * M[0,0] + M[1,0] * M[1,0])

            if sy < 1e-6:
                x = math.atan2(M[2,1] , M[2,2])
                y = math.atan2(-M[2,0], sy)
                z = math.atan2(M[1,0], M[0,0])
            else :
                x = math.atan2(-M[1,2], M[1,1])
                y = math.atan2(-M[2,0], sy)
                z = 0
            print(np.array([x, y, z]))
    else:
        print("Not enough matches %d/%d" % (len(good), MIN_MATCH_CT))
        matchesMask = None

    if cv2.waitKey(5) & 0xFF == 27:
        break

    cv2.imshow("cake", frame)

