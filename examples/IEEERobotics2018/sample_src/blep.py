#!/usr/bin/python3
import cv2
import numpy as np
from ctypes import cdll, c_int

lib = cdll.LoadLibrary("./sumArr.so")
def getCenter(pts):
    return lib.getCenter((c_int * len(pts))(*pts), len(pts))

cap = cv2.VideoCapture(0)

while True:
    _, img = cap.read()
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(grey, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    out = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    for i, row in enumerate(thresh):
        x = getCenter(row)
        if x != -1:
            cv2.circle(out, (x, i), 1, (0, 255, 0), thickness=1)
    cv2.imshow('Binary',out)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cv2.destroyAllWindows()
