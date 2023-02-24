#! /usr/bin/env python

# ROS
import rospy
from rospkg import RosPack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs import WrenchStamped
from frasier_openpose.srv import DetectPoses

# Python
import sys
import os
import cv2
import time

# Frasier Utilities
frasier_utilities_path_src = os.path.join(RosPack().get_path('frasier_utilities'), 'src')
sys.path = [frasier_utilities_path_src] + sys.path
from frasier_utils import Arm, Base, Gripper, Head, Speech

# Pose Processor
op_path = os.path.join(RosPack().get_path('frasier_openpose'), 'src')
sys.path = [op_path] + sys.path
import pose_processor as PP

USE_HSR = rospy.get_param("/receptionist_node/use_hsr", True)
INPUT_IMAGE_TOPIC = '~input_image_topic'

NAMES = ["Nick", "Joanne", "Danny"]
DRINKS = ["Coke", "Orange Juice", "Coffee"]

class Receptionist(object):
    def __init__(self):
        self.speech = Speech(use_hsr=USE_HSR)
        self.head = Head()
        
        self.cv_bridge = CvBridge()
        self.imgmsg_to_cv2 = self.cv_bridge.imgmsg_to_cv2
        self.cv2_to_imgmsg = self.cv_bridge.cv2_to_imgmsg
        self.frame = None


        # Image Subscriber
        self.image_sub = rospy.Subscriber(INPUT_IMAGE_TOPIC, Image, self.image_cb)
        # Wrist force subscriber
        self.wrist_sub = rospy.Subscriber("/hsrb/wrist_wrench/compensated", 1, self.wrist_cb)

        # Speech Subscriber
        self.speech_sub = rospy.Subscriber('/frasier_dialogflow/text', String, self.speech_cb)

        # Publishers
        self.vis_pub = rospy.Publisher('/vis', Image, queue_size=1)

        # Wait for service to be available (MUST LAUNCH PYOPENPOSE SERVER.PY)
        rospy.loginfo('[find_mates_node]: Waiting for pose detection server...')
        rospy.wait_for_service('/frasier_openpose/detect_poses')
        self.pose_cli = rospy.ServiceProxy('/frasier_openpose/detect_poses', DetectPoses)
        rospy.loginfo('[find_mates_node]: Pose detection server ready!')

        self.wrist_force = 0
        self.count = 0

    def say(self, text, blocking=True):
        self.speech.say(text, blocking)

    def image_cb(self, msg):
        # self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.frame = msg

    def speech_cb(self, msg):
        self.sentence = msg.data
        self.result_rcvd = True
        rospy.loginfo("Sentence: {}".format(self.sentence))

    def wrist_cb(self, msg):
        self.wrist_force = msg.wrench.force.x

    def open_door(self):
        self.say ("Please open the door for me, press my wrist when you are done.")

    def confirm_msg(self, msg, count=0):
        ## msg is name or drink
        if count >= 2:
            self.say('I need help.')
            return None
        elif count > 0:
            self.say('Sorry, please repeat your' + msg)

        # Wait for a response
        while not self.result_rcvd: # TODO add timeout
            pass
        self.result_rcvd = False
        sentence = self.sentence

        # Extract name
        for name in NAMES:
            if name not in sentence: 
                return self.confirm_msg(count+1)
            else:
                self.say('%s, right?' % name)
                response = self.detect_yes_no()
                if response is not None and response:
                    self.say('OK')
                    return name
                else:
                    return self.confirm_name(count+1)

    def confirm_msg(self, count=0, msg, msg_list, debug=True):
        ## msg is either name or drink
        if count >= 3:
            self.say('I need help.')
            return None
        elif count > 0:
            self.say('Sorry, please repeat your ' + msg)

        # Wait for a response
        while not self.result_rcvd: # TODO add timeout
            pass
        self.result_rcvd = False
        sentence = self.sentence

        # Extract name
        msg_matched = False
        for m in msg_list:
            if m in sentence:
                msg_matched = True
                break
        if not msg_matched:
            if debug: rospy.loginfo('Msg not found')
            return self.confirm_msg(count+1)

        # We got a name!    
        self.say('%s, right?' % m)
        response = self.detect_yes_no()
        if response is not None and response:
            self.say('OK')
            return m
        else:
            if debug: rospy.loginfo('Perceived msg was incorrect')
            return self.confirm_msg(count+1)

     def detect_yes_no(self, timeout=5): 
        rospy.loginfo('[receptionist_node] Scanning for nodding or shaking...')
        nose_position_history = list()

        endtime = time.time()+timeout

        while time.time() < endtime:
            response = self.pose_cli(self.frame)
            detections = response.detections
            if not detections:
                continue

            vis = self.imgmsg_to_cv2(response.vis_img, "bgr8")
            vis = cv2.flip(vis, 1)
            vis_img = self.cv2_to_imgmsg(vis, "bgr8")
            self.vis_pub.publish(vis_img)

            h, w = self.frame.height, self.frame.width
            centermost_detection = PP.get_centermost_detection(detections, h, w)
            
            nose_position_history.insert(0, PP.log_nose_position(centermost_detection))
            nod_detected = PP.is_detection_nodding(centermost_detection, nose_position_history)
            if nod_detected:
                rospy.loginfo('[receptionist_node] Nodding Head!')
                return True
            shake_detected = PP.is_detection_shaking_head(centermost_detection, nose_position_history)
            if shake_detected:
                rospy.loginfo('[receptionist_node] Shaking Head!')
                return False
        return None

    def get_to_know_guest(self):
        ## TASK: ASK THE PERSON'S NAME + FAVORITE DRINK. CAN ASK TO REPEAT NAME IF WE DO NOT UNDERSTAND
        ## CAN ASK REFEREE TO OPEN DOOR FOR GUEST 
        name = confirm_msg(self, count=0, 'name', NAMES, debug=True)
        drink = confirm_msg(self, count=0, 'drink', DRINKS, debug=True)
        return name, drink
        ## walk to living room area

    def find_empty_seat(self):
        # waypoints for sets, maskrcnn/ openpose to find empty seat
   
    def introduce_guest(self):
        ## POINT AT THE PERSON BEING INTRODUCED (HSR TURN AROUND UNTIL YOU SEE THE PERSON?)
        ## STATE HER NAME AND FAVORITE DRINK
        print ("Everyone, this is <>, and their favorite drink is <> ")

    def receptionist(self):
        while self.count < 3:
            open_door()
            if self.wrist_force > 15:
                name, drink = get_to_know_guest();
                escort_guest()
                find_empty_seat()
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('receptionist_node')
    RC = Receptionist()

    RC.say('Please say, HSR, followed by your command.')

    receptionist()
