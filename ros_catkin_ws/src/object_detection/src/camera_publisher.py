#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

cap = cv2.VideoCapture(0)
img = None

def publisher():
    global cap, img
    pub = rospy.Publisher("camera_image", Image, queue_size=1) 
    rospy.init_node("camera_publisher")
    bridge = CvBridge()

    while not rospy.is_shutdown():
        for _ in range(3):
            ret, img = cap.read()
        if not ret:
            print("CAMERA ERROR: COULD NOT CAPTURE FRAME")
            cap.release()
            time.sleep(0.5)
            cap = cv2.VideoCapture(0)
            continue
        img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        img = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(img)

def main():
    global cap
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()

if __name__ == '__main__':
    main()
