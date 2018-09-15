import cv2 as c
f=c.VideoCapture(0)
while c.waitKey(1)<0:c.imshow('',f.read()[1])
