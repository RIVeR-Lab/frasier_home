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
import numpy as np
import yaml
import yamlordereddictloader
from collections import OrderedDict 

# Robocup
robocup_path = os.path.dirname(RosPack().get_path('frasier_receptionist'))
sys.path, temp = [robocup_path] + sys.path, sys.path
from robocup import RobocupHMI, Robocup
sys.path = temp

names_yaml = os.path.join(robocup_path, 'names.yaml')
with open(names_yaml) as file:
    NAME2GENDER = yaml.load(file)
    NAMES = yaml.load(file)

USE_HSR = rospy.get_param("/receptionist_node/use_hsr", True)
INPUT_IMAGE_TOPIC = '~input_image_topic'

# Frasier Interface YAML
frasier_utilities_path_yaml = os.path.join(RosPack().get_path('frasier_utilities'), 'config')

## User interface commands
html_yaml = os.path.join(frasier_utilities_path_yaml,'html.yaml')
with open(html_yaml) as file:
    HTML_COMMANDS = yaml.load(file)

## Face recognition
frasier_face_recognition_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
known_people_dir = os.path.join(frasier_face_recognition_dir, 'known_people')

## waypoint locations, make waypoints seats we find in the living room
wpt_locations = yaml.load(open(os.path.join(frasier_utilities_path_yaml, 'receptionist_locations.yaml')), Loader=yamlordereddictloader.Loader)

class Receptionist(RobocupHMI):
    
    def __init___(self):
        RobocupHMI.__init___(self)
        self.guests_info = dict()
        self.receptionist_count = 0
        self.guest_count = 0

    def setup_and_wait(self):
        self.arm.goto_position('find_mates')
        self.head.move(pan=0, tilt=-10, degrees=True)
        self.update_interface('Cover my camera to begin', '/hsrb/head_rgbd_sensor/rgb/image_rect_color', True)
        self.say("Cover my camera to begin")
        self.wait_til_covered()

    def travel2start(self):
        pass

    def open_door(self):
        self.update_interface('Please open the door for me. Cover my camera when you are done.', '/hsrb/head_rgbd_sensor/rgb/image_rect_color', True)
        self.say("Please open the door for me. Cover my camera when you are done.")
        self.wait_til_covered()

    def save_person(self, got_name):
        detected_person = self.extract_face()
        cv2.imwrite(os.path.join(known_people_dir, got_name + '.jpg'), detected_person)

    def get_name(self):
        self.new_guest_name = ""
        self.say("Hi. What is your name?")
        instruction = '%s HSR, followed by your name.' % self.add_color('Please say:', 'yellow')
        self.update_interface(instruction)

        self.new_guest_name = self.confirm_name()
        if not self.new_guest_name:
            pass # Use QR code
        return self.new_guest_name

    def get_drink(self):
        self.new_guest_drink = ""
        self.say("Hi. What is your favorite drink?")
        instruction = '%s HSR, followed by your favorite drink.' % self.add_color('Please say:', 'yellow')
        self.update_interface(instruction)

        def confirm_drink(self, count=0, debug=True):
            if count >= 3:
                self.say("I need help.")
                self.update_interface(self.make_pretty(HTML_COMMANDS["help"]))
                return None
            elif count > 0:
                self.say("Sorry, please repeat your favorite drink.")
                self.update_interface(self.make_pretty(HTML_COMMANDS["repeat_drink"]))

            # Wait for a response
            while not self.result_rcvd: # TODO add timeout
                pass
            self.result_rcvd = False
            sentence = self.sentence

            # Extract drink
            drink_matched = False
            for drink in DRINKS:
                if drink in sentence:
                    drink_matched = True
                    break
            if not drink_matched:
                if debug: rospy.loginfo('Drink not found')
                return self.confirm_drink(count+1)

            # We got a drink!    
            self.update_interface(self.make_pretty(self.add_color(drink, "yellow") + HTML_COMMANDS['right'] + "<br>" + HTML_COMMANDS['nod_shake']))
            self.say(drink + ", right?")
            response = self.detect_yes_no()
            if response is not None and response:
                self.say('OK')
                return drink
            else:
                if debug: rospy.loginfo('Perceived drink was incorrect')
                return self.confirm_drink(count+1)

        self.new_guest_drink = self.confirm_drink()
        if not self.new_guest_drink:
            pass # Use QR code
        self.guests_info[new_guest_name] = new_guest_drink
        return self.new_guest_drink

    def escort(self):
        self.update_interface("Please follow me, " + self.add_color(self.new_guest_name, "yellow"))

    def know_guest(self):
        # travel to door
        self.travel2start()
        self.update_interface(RC.make_pretty(HTML_COMMANDS['ready']), '/hsrb/head_rgbd_sensor/rgb/image_rect_color', True)
        self.open_door()
        ## get guest's name and favorite drink
        self.get_name()
        self.save_person()
        self.get_drink()
        ## escort guest 
        self.escort()
        self.guest_count += 1

    def introduce_guests(self, count):
        waypoint_cnt = 0
        found_guests = 0
        while waypoint_cnt < 4:
            if found_guests == self.receptionist_count+1:
                break
            if self.search_for_person():
                rospy.sleep(1.5)
                detected_person = FM.extract_face()
                found_persons = FM.check_known_people(detected_person)
                found_name = found_persons[0]
                found_drink = self.guests_info[found_name]
                ## point to guest
                self.update_interface(self.add_color(found_name + "\'s favorite drink is " + found_drink, "green"))
                self.say("This is " + found_name + ", whose favorite drink is" + found_drink)
                found_guests += 1
            else:
                self.navigate_to(wpt_locations[waypoint_cnt])
                waypoint_cnt += 1
        self.say("No person detected")
    
    def search_for_seat(self):
        self.update_interface("Looking for seats...")
        detected_seats = []
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

    def seat_guest(self):
        # The robot must point at the place or location in which the guest can sit.
        ## maskrcnn to find a seat, waypoints to get us close to seats 
        self.update_interface(self.add_color("Please sit at this chair", "yellow"))
        self.say("This is " + found_name + ", whose favorite drink is" + found_drink)

def main():
    rospy.init_node('receptionist_node')
    RC = Receptionist()
    RC.setup_and_wait()
    ## repeat for 3 new guests
    while RC.receptionist_count < 2: 
        RC.know_guest()
        RC.introduce_guests()
        RC.seat_guest()
        RC.receptionist_count += 1
    
    ## re-seat oldest?


    # RC.head.move(pan=1.2, tilt=0.1)
    # RC.arm.goto_position('bedroom_cub_1')
    # rospy.spin()
    # time.sleep(1.5)
    # RC.nav.go_fwd(0.5, RC.head.pan_heading())

    # name = RC.confirm_name()
    # if name is None:
    #     # TODO set name using a QR code
    #     return 

    # RC.say("I will look for " + name)
    # RC.update_interface(RC.make_pretty(HTML_COMMANDS['look_for_name'] + RC.add_color(name, "yellow")))
    # detections=None
    # while detections is None:
    #     detections = RC.search_for_person()

    # ## Center HSR on face
    # RC.head.move(tilt=0.2)
    # rospy.sleep(0.2)
    # response = RC.pose_cli(RC.frame)
    # detections = response.detections

    # pose = detections[0].pose # Assumes one person in the frame
    # RC.head.center_camera(x=pose.Nose.x/640.0, y=pose.Nose.y/480.0)


if __name__ == '__main__':
    main()

