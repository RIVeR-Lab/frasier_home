#! /usr/bin/env python

# ROS
import rospy
from rospkg import RosPack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from frasier_interface.msg import UpdateInterface
from frasier_openpose.srv import DetectPoses

# Python
import sys
import os
import cv2
import time
import yaml
import numpy as np

# Robocup
robocup_path = os.path.dirname(RosPack().get_path('frasier_find_mates'))
sys.path, temp = [robocup_path] + sys.path, sys.path
from robocup import RobocupHMI, Robocup
sys.path = temp

names_yaml = os.path.join(robocup_path, 'names.yaml')
with open(names_yaml) as file:
    NAME2GENDER = yaml.load(file)
    NAMES = yaml.load(file)

USE_HSR = rospy.get_param("/find_mates_node/use_hsr", True)
INPUT_IMAGE_TOPIC = '~input_image_topic'

# Frasier Interface YAML
frasier_utilities_path_yaml = os.path.join(RosPack().get_path('frasier_utilities'), 'config')

html_yaml = os.path.join(frasier_utilities_path_yaml,'html.yaml')
with open(html_yaml) as file:
    HTML_COMMANDS = yaml.load(file)


class FindMates(RobocupHMI):
    
    def __init___(self):
        RobocupHMI.__init___(self)
        self.names = []

    def setup_and_wait(self):
        self.arm.goto_position('find_mates')
        self.head.move(pan=0, tilt=-10, degrees=True)
        self.update_interface('Cover my camera to begin', '/hsrb/head_rgbd_sensor/rgb/image_rect_color', True)
        self.say("Cover my camera to begin")
        self.wait_til_covered()

    def travel2start(self):
        pass

    def get_names(self):
        self.say("Tell me the names of the mates you would like me to describe.")
        instruction = '%s HSR, please describe NAME1, NAME2, NAME3.' % self.add_color('Say:', 'yellow')
        self.update_interface(instruction)

        def confirm_names(self, count=0, reason=None, debug=True):
            if count >= 3:
                self.say("I need help.")
                self.update_interface(HTML_COMMANDS["help"])
                return False
            elif count > 0:
                self.say("Sorry, please say the names again.")
                self.update_interface(reason+'<br>'+instruction)

            # Wait for a verbal response
            response = self.get_response(timeout=15)
            if response is None: # Didn't hear anything
                self.say("I couldnt hear anything.")
                self.update_interface(self.add_color("I couldnt hear anything.", 'red'))
                return self.confirm_names(count=3) # Immediately give up

            # Extract names
            self.names = [i for i in response.split(' ') if i in NAMES]
            
            # Try again if we didn't hear 3 names
            if len(self.names) != 3:
                self.names = []
                self.confirm_names(count+1, reason='I heard %s names.' % self.add_color(str(len(self.names)), 'red'))

            # Confirm the three names
            txt = '%s, %s, and %s, right?<br>%s' % (self.add_color(self.names[0], 'yellow'),
                                                    self.add_color(self.names[1], 'yellow'),
                                                    self.add_color(self.names[2], 'yellow'),
                                                    HTML_COMMANDS['nod_shake'])
            self.update_interface(txt)
            self.say('%s, %s, and %s, right?' % self.names)
            
            # Scan for head gesture (must be from centermost person)
            response = self.detect_yes_no(timeout=7)
            if response is None:
                self.confirm_names(count+1, reason=self.color("I didn't detect a head nod or shake.", 'red'))
            elif response:
                self.update_interface(self.add_color('You nodded!', 'green'))
                self.say('Great, got it!')
                return True
            else:
                if debug: rospy.loginfo('Perceived names were incorrect')
                return self.confirm_name(count+1, reason=self.add_color('You shook your head.', 'red'))

        got_names = self.confirm_names()
        if not got_names:
            pass # Use QR code

    def travel2search():
        pass



def main():
    rospy.init_node('find_mates_node')
    FM = FindMates()
    FM.setup_and_wait()
    # Cover the camera to proceed
    FM.travel2start()
    FM.get_names()
    FM.travel2search()
    FM.update_interface(FM.make_pretty(HTML_COMMANDS['ready']), '/hsrb/head_rgbd_sensor/rgb/image_rect_color', True)
    FM.say("I'm ready")
    if FM.search_for_person():
        rospy.sleep(1.5)
        detected_person = FM.extract_face()
        if FM.check_known_people(detected_person):
            FM.say("Describing person now")
    else:
        FM.say("No person detected")
    
    # FM.head.move(pan=1.2, tilt=0.1)
    # FM.arm.goto_position('bedroom_cub_1')
    # rospy.spin()
    # time.sleep(1.5)
    # FM.nav.go_fwd(0.5, FM.head.pan_heading())

    # name = FM.confirm_name()
    # if name is None:
    #     # TODO set name using a QR code
    #     return 

    # FM.say("I will look for " + name)
    # FM.update_interface(FM.make_pretty(HTML_COMMANDS['look_for_name'] + FM.add_color(name, "yellow")))
    # detections=None
    # while detections is None:
    #     detections = FM.search_for_person()

    # ## Center HSR on face
    # FM.head.move(tilt=0.2)
    # rospy.sleep(0.2)
    # response = FM.pose_cli(FM.frame)
    # detections = response.detections

    # pose = detections[0].pose # Assumes one person in the frame
    # FM.head.center_camera(x=pose.Nose.x/640.0, y=pose.Nose.y/480.0)


if __name__ == '__main__':
    main()

