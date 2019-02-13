#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def publisher():
    pub = rospy.Publisher("camera_image", Image, queue_size=1) 
    rospy.init_node("camera_publisher", anonymous=True)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        ret, img = cap.read()
        img = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(img)
        rate.sleep()

def main():
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
