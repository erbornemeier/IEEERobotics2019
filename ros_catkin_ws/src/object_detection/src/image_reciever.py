#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(data):
    image = bridge.imgmsg_to_cv2(data,"bgr8")
    print("got image")
    cv2.imshow('test', image)
    cv2.waitKey(1)

def receiver():
    rospy.init_node('image_reciever', anonymous=True)
    rospy.Subscriber('camera_image', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    receiver()
