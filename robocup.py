#! /usr/bin/env python

# ROS
import rospy
from rospkg import RosPack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from frasier_interface.msg import UpdateInterface
from frasier_openpose.srv import DetectPoses
from frasier_face_recognition.srv import RecognizeFace

# Python
import sys
import os
import cv2
import time
import yaml
import numpy as np

# Frasier Utilities
frasier_utilities_path_src = os.path.join(RosPack().get_path('frasier_utilities'), 'src')
sys.path = [frasier_utilities_path_src] + sys.path
from frasier_utils import Arm, Base, Gripper, Head, Speech

# Frasier Nav Client
frasier_nav_path_scripts = os.path.join(RosPack().get_path('frasier_nav_client'), 'scripts')
sys.path = [frasier_nav_path_scripts] + sys.path
from nav_client import NavClient

# Pose Processor
op_path = os.path.join(RosPack().get_path('frasier_openpose'), 'src')
sys.path = [op_path] + sys.path
import pose_processor as PP

# Frasier Interface YAML
frasier_utilities_path_yaml = os.path.join(RosPack().get_path('frasier_utilities'), 'config')
html_yaml = os.path.join(frasier_utilities_path_yaml,'html.yaml')

with open(html_yaml) as file:
    HTML_COMMANDS = yaml.load(file)

# Robocup names
robocup_path = os.path.dirname(os.path.abspath(__file__))
names_yaml = os.path.join(robocup_path, 'names.yaml')
with open(names_yaml) as file:
    NAME2GENDER = yaml.load(file)
    NAMES = NAME2GENDER.keys()


USE_HSR = rospy.get_param("/find_mates_node/use_hsr", True)
INPUT_IMAGE_TOPIC = '~input_image_topic'

DEPTH2INCH = 42.0/1162.0

class Robocup(object):
    def __init__(self):
        self.speech = Speech(use_hsr=USE_HSR)
        self.head = Head()
        self.arm = Arm()
        self.nav = NavClient()
        
        self.cv_bridge = CvBridge()
        self.imgmsg_to_cv2 = self.cv_bridge.imgmsg_to_cv2
        self.cv2_to_imgmsg = self.cv_bridge.cv2_to_imgmsg
        self.frame = None

        # Image Subscriber
        self.image_sub = rospy.Subscriber(INPUT_IMAGE_TOPIC, Image, self.image_cb)

        # Speech Subscriber
        self.speech_sub = rospy.Subscriber('/frasier_dialogflow/text', String, self.speech_cb)
        self.result_rcvd = False
        self.sentence = ''

        # Visualization Publisher
        self.vis_pub = rospy.Publisher('/robocup/vis', Image, queue_size=1)

        # Frasier Interface Publisher
        self.interface_pub = rospy.Publisher('/frasier_interface/update_interface', UpdateInterface, queue_size=1)


    def say(self, text, blocking=True):
        self.speech.say(text, blocking)

    def update_interface(self, text='', image_topic='', flip=True):
        text = self.make_pretty(text)
        msg = UpdateInterface(image_topic=image_topic, flip=flip, text=text)
        self.interface_pub.publish(msg)

    def image_cb(self, msg):
        # self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.frame = msg

    def wait_til_covered(self):
        while True:
            if self.frame is not None:
                img = self.cv_bridge.imgmsg_to_cv2(self.frame, "bgr8")
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if np.sum(img) < 2700000: # Almost completely black
                    return

    def speech_cb(self, msg):
        self.sentence = msg.data
        self.result_rcvd = True
        rospy.loginfo("Sentence: {}".format(self.sentence))

    def get_response(self, timeout=None):
        start_time = time.time()
        while not self.result_rcvd: # TODO add timeout
            if timeout is not None and time.time() > start_time+timeout:
                return None

        self.result_rcvd = False
        return self.sentence

    def make_pretty(self, txt):
        txt = txt.replace("</color>", "</div>").replace("<color=", "<div style='-webkit-text-fill-color: ").replace("/>", "; display:inline'>")
        return txt

    def add_color(self, txt, color):
        msg = "<div style='-webkit-text-fill-color: %s; display:inline'>%s</div>" % (color, txt)
        return msg


class RobocupHMI(Robocup):
    def __init__(self):
        Robocup.__init__(self) 
        # Wait for service to be available (MUST LAUNCH PYOPENPOSE SERVER.PY)
        rospy.loginfo('[find_mates_node]: Waiting for pose detection server...')
        rospy.wait_for_service('/frasier_openpose/detect_poses')
        self.pose_cli = rospy.ServiceProxy('/frasier_openpose/detect_poses', DetectPoses)
        rospy.loginfo('Pose detection server ready!')

        rospy.loginfo('[find_mates_node]: Waiting for face recognition server...')
        rospy.wait_for_service('/frasier_face_recognition/recognize_face')
        self.fr_cli = rospy.ServiceProxy('/frasier_face_recognition/recognize_face', RecognizeFace)
        rospy.loginfo('face recognition server ready!')

        self.depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw', Image, self.depth_cb)


    def depth_cb(self, msg):
        self.depth_frame = self.imgmsg_to_cv2(msg, 'passthrough')

    def get_depth (self, depth_frame, x, y):
        return float(depth_frame[y,x]) * DEPTH2INCH

    def search_for_person(self):
        self.update_interface("Looking for people...")
        detected_people = []
        pan = 90
        self.head.move(pan=pan, tilt=-30, degrees=True)
        rospy.sleep(1.5)        
        
        while pan > -90:
            depth_img  = self.depth_frame
            response   = self.pose_cli(self.frame)
            detections = response.detections
            
            if len(detections) > 0:
                for detection in detections:
                    pose = detection.pose
                    if (pose.RHip.conf > 0.2 and 
                        pose.LHip.conf > 0.2 and 
                        pose.RKnee.conf > 0.2 and 
                        pose.LKnee.conf > 0.2):

                        depths = [self.get_depth(depth_img, i.x, i.y) for i in [pose.RHip, pose.LHip, pose.RKnee, pose.LKnee]]
                        depth = np.median(depths)
                        # self.update_interface(str(depth)[:4])
                        detected_person = dict(detection=detection, depth=depth, pan=pan)
                        detected_people.append(detected_person)

            pan -= 30
            self.head.move(pan=pan, tilt=-30, degrees=True)
            rospy.sleep(0.2)


        min_depth = None
        closest_detection = None
        for i in detected_people:
            if min_depth is None or i['depth'] < min_depth:
                min_depth = i['depth']
                closest_detection = i['detection']
                closest_pan = i['pan']
        if closest_detection is None:
            return False
        x = np.mean([closest_detection.pose.RHip.x, closest_detection.pose.LHip.x])
        self.head.center_camera(x=x, rgbd=True, ghost_pan=closest_pan, ghost_degrees=True, move_time=1.2, tilt_goal=0.1)

        return True

    def extract_face(self, debug=True):
        self.say("Detecting familar faces")
        self.update_interface(self.make_pretty(HTML_COMMANDS['recognize']))
        imgmsg = self.frame
        response = self.pose_cli(imgmsg)
        center_detection = PP.get_centermost_detection(response.detections, imgmsg.height, imgmsg.width)
        face_box = PP.get_face_box(center_detection, imgmsg.height, imgmsg.width)
        if face_box is None:
            vis = self.imgmsg_to_cv2(imgmsg, "bgr8")
            vis = cv2.flip(vis, 1)
            vis_img = self.cv2_to_imgmsg(vis, "bgr8")
            self.vis_pub.publish(vis_img)
            if debug:
                print 'No faces detected'
            return None

        img = self.imgmsg_to_cv2(imgmsg, "bgr8")
        face_min_x, face_min_y, face_max_x, face_max_y = face_box
        img = img[face_min_y:face_max_y, face_min_x:face_max_x]
        self.vis_pub.publish(self.cv2_to_imgmsg(img, "bgr8"))
        return self.cv2_to_imgmsg(img, "bgr8")

    def check_known_people(self, imgmsg):
        response = self.fr_cli(imgmsg)
        found_persons = response.found_persons
        if not found_persons:
            return None
        self.say("I found " + found_persons[0])
        self.update_interface(self.make_pretty(HTML_COMMANDS['found'] + self.add_color(found_persons[0], "yellow")))
        return found_persons

    def confirm_name(self, count=0, debug=True):
        if count >= 3:
            self.say("I need help.")
            self.update_interface(self.make_pretty(HTML_COMMANDS["help"]))
            return None
        elif count > 0:
            self.say("Sorry, please say the name again.")
            self.update_interface(self.make_pretty(HTML_COMMANDS["couldnt_hear"]))

        # Wait for a response
        while not self.result_rcvd: # TODO add timeout
            pass
        self.result_rcvd = False
        sentence = self.sentence

        # Extract name
        name_matched = False
        for name in NAMES:
            if name in sentence:
                name_matched = True
                break
        if not name_matched:
            if debug: rospy.loginfo('Name not found')
            return self.confirm_name(count+1)

        # We got a name!    
        self.update_interface(self.make_pretty(self.add_color(name, "yellow") + HTML_COMMANDS['right'] + "<br>" + HTML_COMMANDS['nod_shake']))
        self.say(name + ", right?")
        response = self.detect_yes_no()
        if response is not None and response:
            self.say('OK')
            return name
        else:
            if debug: rospy.loginfo('Perceived name was incorrect')
            return self.confirm_name(count+1)

    def detect_yes_no(self, timeout=None): 
        rospy.loginfo('[robocup_hmi] Scanning for nodding or shaking...')
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
                rospy.loginfo('[robocup_hmi] Nodding Head!')
                return True
            shake_detected = PP.is_detection_shaking_head(centermost_detection, nose_position_history)
            if shake_detected:
                rospy.loginfo('[robocup_hmi] Shaking Head!')
                return False
        return None