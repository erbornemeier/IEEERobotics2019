
import numpy as np
import cv2

MIN_MATCH_COUNT = 10

target = cv2.imread('map.png', 0)
cap = cv2.VideoCapture(1)

sift = cv2.sift()

FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
    
    _, frame = cap.read()

    kp1, des1 = sift.detectAndCompute(target, None)
    kp2, des2 = sift.detectAndCompute(frame, None)

    matches = flann.knnMatch(des1, des2, k=2)

    good = []
    for m,n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx] for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx] for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = target.shape()
        pts = np.float32([[0,0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspetiveTransform(pts, M)

        frame = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
    else:
        print("Not enough matches found: {}/{}".format(len(good), MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor = (0, 255, 0),
                       singlePointColor = None,
                       matchesMask = matchesMask,
                       flags = 2)

    outImg = cv2.drawMatches(target, kp1, frame, kp2, good, None, **draw_params)

    plt.imshow(outImg, 'match?')
    plt.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
