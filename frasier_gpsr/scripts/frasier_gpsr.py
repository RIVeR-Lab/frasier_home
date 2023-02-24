#!/usr/bin/env python

# ROS
import rospy
import roslaunch
from std_srvs.srv import Empty
from std_msgs.msg import String


# Frasier
from frasier_nlp.msg import *
from frasier_nlp.srv import *
from darknet_ros.srv import *
from utils.hsr_clients import SpeechClient, HeadClient  # Speech
from frasier_nav_client.srv import NavigateTo, NavigateToRequest  # Navigation
from utils import promptrespond  # Respond to 'prompts' or questions
from utils import lookfor
from utils import pose_analysis
from utils import speak
from utils import onscreen
from frasier_manipulation_msgs.srv import GraspObject
from frasier_person_description.srv import DescribePeople, DescribePeopleRequest  # Person description
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Python
from threading import Thread
from pyzbar import pyzbar

LOAD_NETS = True
# LOAD_NETS = False

PROHIBITED_TERMS = ['follow','pose']

BEACONS = ['entrance','cupboard','storage table','end table','couch','dining table','bookcase','side table','sink','dishwasher','desk','bed']

# MUST RUN
# python -m SimpleHTTPServer
# in scripts/utils/templates

class FRASIERGPSR:
    def __init__(self):
        self.abort = False
        self.offer = None
        # Speech parameters
        self.sentence = ''
        self.action = ''
        self.result_rcvd = False
        self.listening_for_response = False

        # Speech client
        self.speech = SpeechClient()

        rospy.sleep(60)
        self.speech.say("Please open the door")
        rospy.sleep(60)
        rospy.wait_for_service('/frasier_navigation/nav_cmd')
        self.nav_cli = rospy.ServiceProxy('/frasier_navigation/nav_cmd', NavigateTo)
        self.execute_navigation('gpsr_start')

        # Head client
        self.head = HeadClient()

        rospy.loginfo("GPSR starting up...")


        rospy.wait_for_service('/frasier/get_robotified_tasks')
        self.robotify_cli = rospy.ServiceProxy('/frasier/get_robotified_tasks', GetRobotifiedTasks)

        rospy.wait_for_service('/frasier/gpsr/grasp')
        self.grasp_cli = rospy.ServiceProxy('/frasier/gpsr/grasp', GraspObject)

        rospy.wait_for_service('/frasier/gpsr/give')
        self.give_cli = rospy.ServiceProxy('/frasier/gpsr/give', Empty)

        rospy.wait_for_service('/frasier/gpsr/drop')
        self.drop_cli = rospy.ServiceProxy('/frasier/gpsr/drop', Empty)

        # Speech Subscriber
        rospy.Subscriber('/frasier_dialogflow/text', String, self.speech_cb)
        # Setup image topic
        self.cv_bridge = CvBridge()
        self.frame = None
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)

        # Initialize recorded_result
        self.recorded_result = "I couldn't find it."
        
        rospy.wait_for_service('/frasier_person_description/describe_people')
        self.describe_cli = rospy.ServiceProxy('/frasier_person_description/describe_people', DescribePeople)
        
        rospy.loginfo("GPSR started")
        self.speech.say('Neural networks are fired up and ready to go!')
        # onscreen.subtitles(self,'Neural networks are fired up and ready to go!')
        self.speech.say('Please say, HSR, followed by your command, Or show me the QR code.')

    def rgb_cb(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True
        except CvBridgeError, e:
            rospy.loginfo(e)

        barcodes = pyzbar.decode(self.frame)
        if len(barcodes) > 0:
            barcodeData = barcodes[0].data.decode("utf-8")
            barcodeDataTest = ''.join([i for i in barcodeData if not i.isdigit()])
            if len(barcodeDataTest) > 5:
                self.barcode_command = barcodeData
                self.barcode_detected = True
        else:
            self.barcode_detected = False 

    # Callbacks
    def speech_cb(self, msg):
        self.sentence = msg.data
        rospy.loginfo("Sentence: {}".format(self.sentence))
        self.result_rcvd = True

    def execute_navigation(self, location, pose=None):
        rospy.loginfo('Starting a nav task...')
        rospy.wait_for_service('/frasier_navigation/nav_cmd')
        rospy.loginfo("Location: {}".format(location))
        try:
            self.speech.say("Going to the {}".format(location.replace('_person','')))
            navigate_to = rospy.ServiceProxy('/frasier_navigation/nav_cmd', NavigateTo)
            req = NavigateToRequest(location=location)
            result = navigate_to(req)
            if result.success:
                self.speech.say("We have arrived")
            else:
                self.speech.say("I am unable to go to the {}".format(location))
        except rospy.ServiceException, e:
            rospy.loginfo("Frasier GPSR: Execute Navigation failed!\n {}".format(e))

    def execute_talking(self, sentence):
        if sentence == 'recorded_result':
            self.speech.say(self.recorded_result)
        else:
            response = promptrespond.get_answer_to_closest_prompt(sentence)
            self.speech.say(response)

    def execute_grasping(self, object_to_grasp):
        req = GraspObject()
        req.object_name = 'test'
        txt = 'Please push on my hand when the object is between my claws, so that I can grasp it.'
        self.speech.say(txt)
        res = self.grasp_cli(req)
        if not res.success:
            self.speech.say("Sorry, I couldn't grasp the {}".format(object_to_grasp))

    def execute_giving(self):
        txt = 'Please pull my hand for me to release the object.'
        self.speech.say(txt)
        self.give_cli()

    def execute_abort(self):
        txt = 'I am not capable of performing the next action.'
        self.speech.say(txt)
        self.abort = True

    def execute_escort(self,task):
        name = task.who_to_escort
        txt = 'OK '+name+', please follow behind me.'
        self.speech.say(txt)
        self.execute_navigation(task.where_to_escort)

    def execute_drop(self):
        self.drop_cli()

    def execute_lookfor(self, task):
        if task.type == 'object':
            found = lookfor.lookfor_objects(self, task, tilt=0)
            if not found:
                self.recorded_result = 'Hello operator. I could not find the right object.'
        elif task.type == 'human':
            found = lookfor.lookfor_humans(self, task)
            if not found:
                self.recorded_result = 'Hello operator. I could not find the right person.'

    def execute_answerquestion(self):
        txt = "I'm here to answer your question."
        self.speech.say(txt)
        # self.speech.say('Please show me a QR code of your question.')
        # while not self.barcode_detected: pass
        # question = self.barcode_command
        question = speak.get_response(self, only_last_word=False)
        answer = promptrespond.get_answer_to_closest_question(question)
        self.speech.say(answer)

    def robotify_speech_command(self):  # TODO: Anas + Tarik
        # Create a request
        request = GetRobotifiedTasksRequest()
        resp = RobotifiedTask()
        # Enter robotify loop
        while not rospy.is_shutdown():
            if self.result_rcvd or self.barcode_detected:
                rospy.loginfo('COMMAND HAS BEEN RECEIVED')
                if self.result_rcvd:
                    # Set the sentence to parse
                    sentence = speak.verify_command(self)
                    # sentence = self.sentence
                    self.result_rcvd = False
                else:
                    rospy.loginfo('COMMAND HAS BEEN RECEIVED')
                    sentence = self.barcode_command
                    self.speech.say('Barcode detected')
                    repeat_cmd = sentence.replace(' me ',' you ').replace(' me.',' you.')
                    txt = 'You asked me to '+repeat_cmd
                    self.barcode_detected = False

                if 'drink' in sentence.split() and 'chocolate' not in sentence.split():
                    self.offer = 'drink'
                elif 'eat' in sentence.split():
                    self.offer = 'eat'

                sentence = sentence.lower().replace('offer something to','tell me how many')
                sentence = sentence.replace('to all the ','')
                sentence = sentence.replace('drink ','')
                sentence = sentence.replace('eat ','')
                sentence = sentence.replace('looks like','to me')
                sentence = sentence.replace('to me','to you')
                sentence = sentence.replace('tell me how','describe')
                request.command = sentence
                # Get the response
                response = self.robotify_cli(request)
                tasks = response.tasks

                dests = [task.location_to_go_to for task in tasks]
                # Iterate over all responses and apply execution

                for task_index,task in enumerate(tasks):

                    if self.abort:
                        self.execute_navigation('gpsr_start')
                        break

                    if task.task == resp.LOOKFOR:
                        if tasks[task_index-1].location_to_go_to == 'gpsr_start':
                            continue
                        if tasks[task_index-1].task == resp.LOOKFOR:
                            continue
                        # if tasks[task_index-1].record == 'recorded_result':
                        #     continue
                        self.execute_lookfor(task)
                    elif task.task == resp.NAVIGATION:
                        if tasks[task_index+1].task == resp.LOOKFOR \
                            and task.location_to_go_to in BEACONS: 
                            self.execute_navigation(task.location_to_go_to+'_person')
                        else:
                            self.execute_navigation(task.location_to_go_to)
                    elif task.task == resp.GRASP:
                        self.execute_grasping(task.object_to_grasp)
                    elif task.task == resp.GIVE:
                        self.execute_giving()
                    elif task.task == resp.TELL:
                        if tasks[task_index-1].task != resp.NAVIGATION:
                            self.execute_navigation('gpsr_start')
                            dests.append('gpsr_start')  
                        self.execute_talking(task.what)
                    elif task.task == resp.ANSWER:
                        self.execute_answerquestion()
                    elif task.task == resp.ESCORT:
                        self.execute_escort(task)
                    elif task.task == resp.FOLLOW:
                        self.execute_abort()
                    elif task.task == resp.SELFINTRO:
                        self.speech.say('Hi, my name is Frasier. Nice to meet you.')
                    if task_index == len(tasks)-1 and 'gpsr_start' not in dests:
                        self.execute_navigation('gpsr_start')

                self.speech.say('I am ready for another command.')
                self.speech.say('Please say, HSR, followed by your command.')
                
            rospy.Rate(10).sleep()


if __name__ == '__main__':
    rospy.init_node('frasier_gpsr')
    gpsr = FRASIERGPSR()
    # Create a new thread to start and spin
    robotify_thread = Thread(target=gpsr.robotify_speech_command)
    robotify_thread.start()
    rospy.spin()
