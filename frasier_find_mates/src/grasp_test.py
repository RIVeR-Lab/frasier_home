#!/usr/bin/env python

import rospy
import roslaunch
from rospkg import RosPack
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from frasier_display.msg import WebDisplay
from frasier_research.srv import GraspObjectsRequest, GraspObjects
from mask_rcnn_ros.srv import GetDetectionsRequest, GetDetections

from math import pi
from random import random, randint
import time
from pyzbar import pyzbar
from yaml import load, YAMLError


class GraspTest:

    def __init__(self):

        # Wait for services
        rospy.loginfo("[grasp_test]: Waiting for Mask service...")
        rospy.wait_for_service('/mask_rcnn/get_detections', 5)
        self.mask_service = rospy.ServiceProxy('/mask_rcnn/get_detections', GetDetections)
        rospy.loginfo("Fraiser Tidy Up: Initialized!")

        rospy.loginfo("[grasp_test]: Waiting for Grasp service...")
        rospy.wait_for_service('/frasier_research/grasp_object', 10)
        self.grasp_service = rospy.ServiceProxy('/frasier_research/grasp_object', GraspObjects)
        rospy.loginfo("[grasp_test]: Got Grasp service!")


if __name__ == '__main__':
    rospy.init_node('GraspTestNode')
    GT = GraspTest()
    grasp_resp = GT.grasp_service(GraspObjectsRequest(table=False))