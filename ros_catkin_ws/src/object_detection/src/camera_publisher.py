#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def publisher():
    pub = rospy.Publisher("camera_image", Image, queue_size=1) 
    rospy.init_node("camera_publisher")
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        for _ in range(3):
            ret, img = cap.read()
        img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        img = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(img)

def main():
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
