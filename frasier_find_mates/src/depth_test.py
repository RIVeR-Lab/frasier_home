#!/usr/bin/env python

import rospy
import roslaunch
from rospkg import RosPack
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

DEPTH2INCH = 42.0/1162.0

class DepthTest:

    def __init__(self):
        # Image Subscriber
        self.image_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw', Image, self.image_cb)
        self.cv_bridge = CvBridge()
        self.imgmsg_to_cv2 = self.cv_bridge.imgmsg_to_cv2
        self.cv2_to_imgmsg = self.cv_bridge.cv2_to_imgmsg
        self.frame = None

    def image_cb(self, imgmsg):
        self.frame = self.imgmsg_to_cv2(imgmsg, 'passthrough')

if __name__ == '__main__':
    rospy.init_node('DepthTestNode')
    DT = DepthTest()
    while True:
        if DT.frame is not None:
            print float(DT.frame[240, 320]) * DEPTH2INCH

