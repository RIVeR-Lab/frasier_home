#!/usr/bin/env python

# ROS
import rospy

# Frasier
from frasier_nlp.msg import *
from frasier_nlp.srv import *
from std_msgs.msg import String
from utils import promptrespond, pose_analysis, onscreen
from utils.hsr_clients import SpeechClient, HeadClient, ArmClient  # Speech
from frasier_person_description.srv import DescribePeople, DescribePeopleRequest  # Person description
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Python
import time 

START_CMD = 'analyze the people in front of you'

class FRASIERBRONZE:
    def __init__(self):
        onscreen.subtitles('Getting ready, please be patient!')
        # Speech parameters
        self.sentence = ''
        self.result_rcvd = False

        # Speech client
        self.speech = SpeechClient()
        # Head client
        self.head = HeadClient()
        self.head.move(tilt=-0.25,pan=-1.75)
        # Arm client
        self.arm = ArmClient()
        self.arm.spr_start()
        # Speech Subscriber
        rospy.Subscriber('/frasier_dialogflow/text', String, self.speech_cb)
        # Person Describe Service
        self.describe_cli = rospy.ServiceProxy('/frasier_person_description/describe_people', DescribePeople)
        rospy.wait_for_service('/frasier_person_description/describe_people')
        self.say_with_subs('Neural networks are fired up and ready to go!')

        # Setup image topic
        self.cv_bridge = CvBridge()
        self.frame = None
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)

    # Callbacks
    def speech_cb(self, msg):
        self.sentence = msg.data
        self.result_rcvd = True
        rospy.loginfo("Sentence: {}".format(self.sentence))
    def rgb_cb(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True
        except CvBridgeError, e:
            rospy.loginfo(e)

    def say_with_subs(self, txt):
        onscreen.subtitles(txt)
        self.speech.say(txt)

    def start_analysis(self):
        perceived_cmd = promptrespond.get_closest_list_member(self.sentence, [START_CMD])
        result = self.result_rcvd and perceived_cmd == START_CMD
        self.result_rcvd = False
        self.sentence = ''
        return result

    def capture_ppl(self):
        self.say_with_subs("Ok everyone, please assemble in front of me.")
        self.say_with_subs("Make sure that I can see you.")
        end_time = time.time() + 4
        while time.time() < end_time:
            # onscreen.show_image(self.frame)
            # pose_analysis.center_camera_on_all_people(self,move_time=0.5)
            pass
        self.say_with_subs("I'm ready to take a photo. Say cheeeeese!")
        req = DescribePeopleRequest(gender=True,
                                    age=True,
                                    emotion=True,
                                    upper_fashion=True,
                                    bottom_fashion=True)
        descriptions = self.describe_cli.call(req)
        print descriptions
        txt = pose_analysis.generate_descriptions_text(descriptions)
        self.speech.say(txt)
        txts = pose_analysis.generate_descriptions_summary(descriptions)
        onscreen.show_desc_txts(txts)

if __name__ == '__main__':
    rospy.init_node('frasier_bronze_demo')
    demo = FRASIERBRONZE()

    demo.say_with_subs('Please say, HSR, followed by your command.')

    while not rospy.is_shutdown():
        if demo.start_analysis():
            demo.capture_ppl()
        rospy.sleep(1)