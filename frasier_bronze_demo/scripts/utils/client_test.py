#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from hsr_clients import *

rospy.init_node('arm_test')

head = HeadClient()
head.move(1.5, 0.0, move_time=2)