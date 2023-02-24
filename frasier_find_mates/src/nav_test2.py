#!/usr/bin/env python

# ROS
import rospy
from scipy.spatial.transform import Rotation as R
from frasier_nav_client.srv import NavigateTo, NavigateToRequest  # Navigation
from geometry_msgs.msg import PoseStamped

import os
import sys
import math
import time
import rospy
from rospkg import RosPack

# Frasier Utilities
frasier_utilities_path_src = os.path.join(RosPack().get_path('frasier_utilities'), 'src')
sys.path = [frasier_utilities_path_src] + sys.path
from frasier_utils import Base

class NavTest:

    def __init__(self):
        # rospy.wait_for_service('/frasier_navigation/nav_cmd')
        # self.nav_cli = rospy.ServiceProxy('/frasier_navigation/nav_cmd', NavigateTo)
        rospy.Subscriber('/global_pose', PoseStamped, self.pose_cb)
        self.base = Base()
        # req = NavigateToRequest(location=location)
        # result = navigate_to(req)
        # if result.success:
        #     self.speech.say("We have arrived")
        # else:
        #     self.speech.say("I am unable to go to the {}".format(location))

    def pose_cb(self, msg):
        self.msg = msg

    def gen_fwd_quat(self, fwd_distance, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        heading_rad = R.from_quat([0, 0, z, w]).as_euler('zyx')[0]
        xhat = x+fwd_distance*math.cos(heading_rad)
        yhat = y+fwd_distance*math.sin(heading_rad)



if __name__ == '__main__':
    rospy.init_node('navtest')
    NT = NavTest()
    time.sleep(3)
    # msg = NT.gen_fwd_quat(1, NT.msg)
    NT.base.move_to_relative(1, 1, 1)
    rospy.spin()