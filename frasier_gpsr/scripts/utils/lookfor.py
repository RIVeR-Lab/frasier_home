import rospkg, rospy
from frasier_person_description.srv import DescribePeopleRequest
from darknet_ros.srv import GetDetectionsRequest

import time
import numpy as np

import pose_analysis
import promptrespond
import speak

import sys

pkg_dir = rospkg.RosPack().get_path('frasier_person_description')
sys.path.append(pkg_dir + '/src/deepfashion')
import color_detectv2 as color_detect

import yaml

pkg_dir = rospkg.RosPack().get_path('frasier_gpsr')
yaml_dirname = pkg_dir + '/config/'

GPSR_TO_HEX = {
    'black': u'#000000',
    'blue': u'#0000ff',
    'gray': u'#808080',
    'red': u'#ff0000',
    'white': u'#ffffff',
    'yellow': u'#ffff00',
    'orange': u'#ffa500'
}
HEX_TO_GPSR = {v: k for k, v in GPSR_TO_HEX.iteritems()}

DEEPFASHION_TO_GPSR = yaml.load(open(yaml_dirname + '/deepfashion2gpsr.yaml'))

# People properties
PEOPLE_PROPERTIES = ['tallest', 'smallest', 'oldest', 'youngest', 'slimmest', 'fattest']
MEN_TERMS = ['man', 'men', 'boy', 'boys', 'male person']
WOMEN_TERMS = ['woman', 'women', 'girl', 'girls', 'female person']
GENDER_TERMS = MEN_TERMS + WOMEN_TERMS
AGE_TERMS = ['child', 'children', 'youngest', 'oldest', 'elderly', 'elder', 'elders']
NAMES = [i.replace('\n', '').lower() for i in open(yaml_dirname + '/names.txt').readlines()]
UPPER_FASHION_TERMS = [key for key, value in DEEPFASHION_TO_GPSR.iteritems() if value != 'pants'] + \
                      [value for key, value in DEEPFASHION_TO_GPSR.iteritems() if value != 'pants']
BOTTOM_FASHION_TERMS = [key for key, value in DEEPFASHION_TO_GPSR.iteritems() if value == 'pants'] + ['pants']
COLOR_TERMS = list(GPSR_TO_HEX.keys()) + \
              list(color_detect.HTML4_NAMES_TO_HEX.keys())

EMOTION_TO_CLASSIFIER_OUTPUT = yaml.load(open(yaml_dirname + '/emotion2classifieroutput.yaml'))
EMOTION_TERMS = list(EMOTION_TO_CLASSIFIER_OUTPUT.keys())

POSES = ['standing','lying down','sitting']

FACE_TILT = 0.20
HIGH_FACE_TILT = 0.4
TORSO_TILT = 0
BELOW_WALL_TILT = -0.15
UPPER_TILT_LIMIT = 1.0

DESCRIBE_REQ = DescribePeopleRequest(gender=True,
                                     age=True,
                                     emotion=True,
                                     upper_fashion=True,
                                     bottom_fashion=True)
# Object properties
OBJECT_PROPERTIES = ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']
OBJECT_NAME_TO_CATEGORY = yaml.load(open(yaml_dirname + '/objectname2category.yaml'))
OBJECT_NAME_TO_SIZE = yaml.load(open(yaml_dirname + '/objectname2size.yaml'))
OBJECT_NAME_TO_WEIGHT = yaml.load(open(yaml_dirname + '/objectname2weight.yaml'))
OBJECT_CATEGORIES = OBJECT_NAME_TO_CATEGORY.values()
DARKNET_THRESHOLD = 0.3


def get_criteria_flags(task):
    flags = dict()
    flags['describe'] = task.record == 'describe'
    flags['gender'] = task.name.lower() in GENDER_TERMS or task.record == 'gender'
    flags['age'] = task.name.lower() in AGE_TERMS
    flags['emotion'] = task.adj.lower() in EMOTION_TERMS

    # task.fashion could be color+clothing or just one of the two
    flags['color'] = task.fashion.lower().split()[0] in COLOR_TERMS or flags['describe']
    flags['upper_fashion'] = task.fashion.lower().split()[-1] in UPPER_FASHION_TERMS or flags['describe']
    flags['bottom_fashion'] = task.fashion.lower().split()[-1] in BOTTOM_FASHION_TERMS or flags['describe']

    flags['req'] = DescribePeopleRequest(gender=flags['gender'] or flags['describe'],
                                         age=flags['age'] or flags['describe'],
                                         emotion=flags['emotion'] or flags['describe'],
                                         upper_fashion=flags['upper_fashion'] or flags['color'] or flags['describe'],
                                         bottom_fashion=flags['bottom_fashion'] or flags['color'] or flags['describe'])
    if flags['color']:
        # Then the first word in fashion would be color
        flags['color_to_find'] = task.fashion.split()[0].lower()
    if flags['upper_fashion'] or flags['bottom_fashion']:
        # Then the last word in fashion would be clothing
        flags['clothing_to_find'] = task.fashion.split()[-1].lower()
    # Are we looking for a face?
    flags['face'] = flags['gender'] or flags['age'] or flags['emotion'] or flags['describe']
    # Are we looking for a gesture?
    flags['gesture'] = task.gesture != 'any' or task.record == 'gesture'
    # Are we looking for a pose?
    flags['pose'] = task.posture != 'any' or task.record == 'pose'
    rospy.loginfo('SEARCH FLAGS:')
    rospy.loginfo(flags)
    return flags


def lookfor_humans(self, task):
    if task.record == 'count':
        results = count_humans(self,task)
    else:
        # Do we not need to find the person that best exhibits
        # a trait out of everyone we find? ex. oldest
        # find_only_first_match = not task.adj in PEOPLE_PROPERTIES

        # if find_only_first_match:
        # Just find the first person that matches the criteria
        results = find_first_person(self, task)

    rospy.loginfo('LOOKFOR RESULTS:')
    rospy.loginfo(results)
    if results is None:
        # if find_only_first_match:
        txt = "I couldn't find anyone matching the criteria."
        # else:
            # txt = "I could not determine the " + task.adj + " person."
        self.speech.say(txt)
        return False
    else:
        if task.record == 'count':
            num = str(results)
            txt = 'I found '+num+' people that match the criteria.'
            self.speech.say(txt)
            self.recorded_result = 'Hello operator. '+txt

        elif task.name.lower() in NAMES:
            self.speech.say("Hello " + task.name + "! I've been searching for you.")
        else:
            self.speech.say("Hello there! I've been searching for you.")
        if task.record == 'name':
            name = get_name(self)
            # self.speech.say('Please show me a QR code of your name.')
            # while not self.barcode_detected: pass
            # name = self.barcode_command

            if name is not None:
                self.recorded_result = 'Hello operator. The person you told me to find is named ' + name
            else:
                self.recorded_result = "Hello operator. The person did not tell me their name."
        elif task.record == 'gender':
            assumed_gender = results['guessed_gender']
            # gender = get_gender(self,assumed_gender)
            # if gender is None:
            gender = assumed_gender
            self.speech.say("I found a " + gender)
            self.recorded_result = 'Hello operator. The person you told me to find is a ' + gender
        elif task.record == 'describe':
            txt = generate_descriptions_text(results)
            self.speech.say('I see ' + txt)
            self.recorded_result = 'I saw ' + txt
        return True


def count_humans(self,task):
    txt = 'I will now begin to look for humans. You must be between 1 and 2 meters away from me for me to see you.'
    self.speech.say(txt)
    matches = 0
    # Get into starting position
    tilt = BELOW_WALL_TILT
    self.head.move(tilt=tilt, move_time=1)
    # Set future places to search
    pan_points = [0, 1.2, -1.2]
    for pan_point in pan_points:
        self.head.move(pan=pan_point)
        time.sleep(1.5)
        # Look for people
        pose_msgs = pose_analysis.get_just_poses(self)
        # Skip if no people found
        if pose_msgs is None:
            continue
        # speak.report_num_people_from_pose_msgs(self, pose_msgs)
        # For each person in view,
        for ind, pose_msg in enumerate(pose_msgs):
            rospy.loginfo('Analyzing person #' + str(ind))
            cx,cy,found = pose_analysis.get_face_center(pose_msg)
            pose_analysis.pan_to_pose(self,pose_msg)
            if found:
                rospy.loginfo('Person #' + str(ind) + ' face found! Centering...')
                self.head.center_camera(cx,cy,move_time=0.5)
            else:
                rospy.loginfo('Person #' + str(ind) + ' face missing. Centering on pose instead.')
                face_found = pose_analysis.try_find_face(self)
                if not face_found:
                    txt = "I cannot see this person's face. Could they show me their face?"
                    self.speech.say(txt)
                    end_time = time.time() + 5
                    while not face_found and time.time() < end_time:
                        face_found = pose_analysis.try_find_face(self)
                    if not face_found:
                        txt = "Moving on to the another person."
                        self.speech.say(txt)
                        continue

            flags = get_criteria_flags(task)
            # Now we must be centered on a person!
            # Create dictionary to store any results
            results = dict()
            describe_person(self, results, flags)

            if not matching_description(results,task):
                continue

            matches += 1
            if self.offer is not None:
                txt = "Would you like something to "+self.offer
                self.say.speech(txt)
                txt = "Please say HSR yes, or HSR no."
                self.say.speech(txt)
                speak.get_response(self)

    return matches

def matching_description(results,task):
    flags = get_criteria_flags(task)

    if flags['gender']:
        # Looking for men:
        if task.name.lower() in MEN_TERMS and results.get('guessed_gender', None) != 'man':
            rospy.loginfo('FAILED: NOT A MAN')
            return False
        # Looking for women:
        elif task.name.lower() in WOMEN_TERMS and results.get('guessed_gender', None) != 'woman':
            rospy.loginfo('FAILED: NOT A WOMAN')
            return False
        # Trying to determine gender:
        elif results.get('guessed_gender', None) is None:
            rospy.loginfo('FAILED: COUDLNT GET GENDER')
            return False

    # Check their upper fashion?
    if flags['upper_fashion'] \
            and not check_results_fashion_and_color(flags, 'upper_fashion', results):
        rospy.loginfo('FAILED: NOT THE RIGHT UPPER FASHION')
        return False

    # Check their bottom fashion?
    if flags['bottom_fashion'] \
            and not check_results_fashion_and_color(flags, 'bottom_fashion', results):
        rospy.loginfo('FAILED: NOT THE RIGHT BOTTOM FASHION')
        return False

    # Check for color on any clothing?
    if flags['color']:
        u_hexcode = results.get('guessed_upper_hexcode', '')
        b_hexcode = results.get('guessed_bottom_hexcode', '')
        if are_different_colors(flags['color_to_find'], u_hexcode) \
                and are_different_colors(flags['color_to_find'], b_hexcode):
            rospy.loginfo('FAILED: NOT THE RIGHT COLOR')
            return False
        
    if flags['gesture'] and results.get('gesture',None) != task.gesture:
        rospy.loginfo('FAILED: NOT THE RIGHT GESTURE')
        return False

    if task.posture in ['sitting','lying down'] \
         and results.get('pose',None) != 'sitting':
        rospy.loginfo('FAILED: NOT THE RIGHT POSTURE')
        return False

    if task.posture == 'standing' and results.get('pose',None) != 'standing':
        rospy.loginfo('FAILED: NOT THE RIGHT POSTURE 2')
        return False

    return True

def find_first_person(self, task):
    txt = 'I will now begin to look for humans. You must be between 1 and 2 meters away from me for me to see you.'
    self.speech.say(txt)
    # Get into starting position
    tilt = BELOW_WALL_TILT
    self.head.move(tilt=tilt, move_time=1)
    # Set future places to search
    pan_points = [0, 1.2, -1.2]
    for pan_point in pan_points:
        self.head.move(pan=pan_point)
        time.sleep(1.5)
        # Look for people
        pose_msgs = pose_analysis.get_just_poses(self)
        rospy.loginfo('PEOPLE FOUND FIRST PERSON')
        rospy.loginfo(pose_msgs)
        # Skip if no people found
        if pose_msgs is None:
            continue
        # speak.report_num_people_from_pose_msgs(self, pose_msgs)
        # For each person in view,
        for ind, pose_msg in enumerate(pose_msgs):
            rospy.loginfo('Analyzing person #' + str(ind))
            pose_analysis.pan_to_pose(self,pose_msg)
            cx,cy,found = pose_analysis.get_face_center(pose_msg)
            if found:
                rospy.loginfo('Person #' + str(ind) + ' face found! Centering...')
                self.head.center_camera(cx,cy,move_time=0.5)
            else:
                rospy.loginfo('Person #' + str(ind) + ' face missing. Centering on pose instead.')
                face_found = pose_analysis.try_find_face(self)
                if not face_found:
                    txt = "I cannot see this person's face. Could they show me their face?"
                    self.speech.say(txt)
                    end_time = time.time() + 5
                    while not face_found and time.time() < end_time:
                        face_found = pose_analysis.try_find_face(self)
                    if not face_found:
                        txt = "Moving on to the another person."
                        self.speech.say(txt)
                        continue

            # Now we must be centered on a person!
            # Create dictionary to store any results
            results = dict()
            flags = get_criteria_flags(task)

            describe_person(self, results, flags)

            if task.record == 'describe':
                return results

            if not matching_description(results,task):
                continue            

            return results

    # We never found anyone that matches the criteria.
    return None


def describe_person(self, results, flags):
    rospy.loginfo('GETTING AS MUCH DESCRIPTION')
    self.speech.say("I'm gathering information about you")
    # We should already be horizontally-centered on a person,
    # and we should be pointed to their face.
    tilt = self.head.current_tilt
    # If we need to know the gesture, we need to see above their face
    if flags['pose']:
        if tilt > 0.4:
            results['pose'] = 'standing'
        else:
            results['pose'] = 'sitting'
        flags['pose'] = False
    if flags['gesture']:
        tilt += 0.2
    while tilt > -0.4:
        if search_is_over(flags):
            return True
        self.head.move(tilt=tilt)
        descriptions = self.describe_cli.call(flags['req'])
        center_ind = pose_analysis.get_centermost_pose(descriptions, index=True)
        tilt -= 0.2
        if center_ind is None:
            continue

        if flags['face'] and descriptions.genders[center_ind] != '':
            results['guessed_gender'] = descriptions.genders[center_ind]
            results['guessed_age'] = descriptions.ages[center_ind]
            results['guessed_emotion'] = descriptions.emotions[center_ind]
            flags['req'] = change_req_input(flags['req'], face=False)
            flags['face'] = False

        if flags['upper_fashion'] and descriptions.upper_fashions[center_ind] != '':
            results['guessed_upper_fashion'] = descriptions.upper_fashions[center_ind].lower()
            results['guessed_upper_hexcode'] = descriptions.upper_hexcodes[center_ind]
            flags['req'] = change_req_input(flags['req'], upper_fashion=False)
            flags['upper_fashion'] = False

        if flags['bottom_fashion'] and descriptions.bottom_fashions[center_ind] != '':
            results['guessed_upper_fashion'] = descriptions.bottom_fashions[center_ind].lower()
            results['guessed_upper_hexcode'] = descriptions.bottom_hexcodes[center_ind]
            flags['req'] = change_req_input(flags['req'], bottom_fashion=False)
            flags['bottom_fashion'] = False

        if flags['gesture']:
            txt = 'Please make sure that I can see your gestures.'
            self.speech.say(txt)
            results['gesture'] = pose_analysis.get_gesture_of_centermost(self)
            txt = "This person seems like they're "+results['gesture']
            self.speech.say(txt)
            flags['gesture'] = False

        if flags['color']:
            u_hexcode = results.get('guessed_upper_hexcode', '')
            b_hexcode = results.get('guessed_bottom_hexcode', '')
            if not (are_different_colors(flags['color_to_find'], u_hexcode) \
                    and are_different_colors(flags['color_to_find'], b_hexcode)):
                flags['req'] = change_req_input(flags['req'], upper_fashion=False)
                flags['req'] = change_req_input(flags['req'], bottom_fashion=False)



    return True


def search_is_over(flags):
    return not (flags['face'] or flags['upper_fashion'] or flags['bottom_fashion'] or flags['pose'] or flags['gesture'])


def generate_descriptions_text(results):
    txt = ''

    if 'guessed_gender' in results:
        lower_age, upper_age = results['guessed_age'].split(',')

        txt += 'a ' + results['guessed_gender'] + ' '
        txt += 'between the ages of ' + lower_age + ' and ' + upper_age + ' '
        txt += 'with a ' + results['guessed_emotion'] + ' expression '
    else:
        txt += 'a person'
    if 'guessed_upper_fashion' in results:
        clothing = results['guessed_upper_fashion']
        color = color_detect.closest_colour(results['guessed_upper_hexcode'])
        fashion = color + ' ' + clothing
        txt += 'wearing a ' + fashion + ' '
        if 'guessed_bottom_fashion' in results:
            clothing = results['guessed_bottom_fashion']
            color = color_detect.closest_colour(results['guessed_bottom_hexcode'])
            fashion = color + ' ' + clothing
            txt += 'and a ' + fashion + ' '
    elif 'guessed_bottom_fashion' in results:
        clothing = results['guessed_bottom_fashion']
        color = color_detect.closest_colour(results['guessed_bottom_hexcode'])
        fashion = color + ' ' + clothing
        txt += 'wearing ' + fashion + ' '

    print txt
    return txt


def check_results_fashion_and_color(flags, fashion_type, results):
    if fashion_type == 'upper_fashion':
        clothing = results.get('guessed_upper_fashion', None)
    else:
        clothing = results.get('guessed_bottom_fashion', None)
    if clothing is None or are_different_clothes(flags['clothing_to_find'], clothing):
        return False
    if flags['color']:
        if fashion_type == 'upper_fashion':
            hexcode = results.get('guessed_upper_hexcode', None)
        else:
            hexcode = results.get('guessed_bottom_hexcode', None)
        if hexcode is None or are_different_colors(flags['color_to_find'], hexcode):
            return False
    return True


def are_different_clothes(clothing_to_find, clothing):
    return clothing_to_find != clothing and clothing_to_find != DEEPFASHION_TO_GPSR.get(clothing, None)


def are_different_colors(color_to_find, hexcode):
    return color_to_find != color_detect.closest_colour(hexcode) \
           and color_to_find != color_detect.closest_colour(hexcode, color_dict=HEX_TO_GPSR)


def change_req_input(req, face=None, upper_fashion=None, bottom_fashion=None):
    if face is not None:
        req.gender = face
        req.age = face
        req.emotion = face
    if upper_fashion is not None:
        req.upper_fashion = upper_fashion
    if bottom_fashion is not None:
        req.bottom_fashion = bottom_fashion
    return req


def get_name(self):
    txt = 'I would like to know your name. Please say, HSR my name is, followed by your name.'
    self.speech.say(txt)
    # 3 attempts maximum
    for _ in xrange(4):
        name = speak.get_response(self)
        if name == '':
            return ''
        self.speech.say("Your name is " + name + ", right?")
        correction = speak.is_correct(self)
        if correction is None:
            txt = "Sorry, I didn't catch that. Please say HSR that's correct, or HSR that's incorrect"
            self.speech.say(txt)
        elif correction:
            self.speech.say('Thank you. Nice to meet you ' + name)
            return name
        else:
            txt = 'I apologize. One more time please say HSR my name is, followed by your name.'
            self.speech.say(txt)

    return None


def get_gender(self, assumed_gender):
    txt = "I was told to find out your gender. Excuse me if I'm wrong, but according to my neural network, you're a " + \
          assumed_gender + ", correct? Please say, HSR that's correct, or HSR that's incorrect"
    self.speech.say(txt)
    # 3 attempts maximum
    for _ in xrange(3):
        response = speak.get_response(self).split()[-1]
        if response == '':
            return ''
        if response == 'correct':
            txt = 'Understood. Thank you.'
            self.speech.say(txt)
            return assumed_gender
        elif response == 'incorrect':
            txt = 'Understood, apologies for the mixup. Thank you.'
            self.speech.say(txt)
            gender = 'man' if assumed_gender == 'woman' else 'woman'
            return gender
        else:
            txt = "Sorry, I didn't catch that. Please say HSR that's correct, or HSR that's incorrect"
            self.speech.say(txt)
    return None


# LOOKING FOR OBJECTS

def lookfor_objects(self, task, tilt=0):
    task.name = promptrespond.get_closest_list_member(task.name,
                                                      OBJECT_NAME_TO_CATEGORY.keys() + OBJECT_NAME_TO_CATEGORY.values())

    # Do we need to find the object that best exhibits
    # a trait out of everything we find? ex. biggest
    find_only_best = task.record in OBJECT_PROPERTIES  # TODO: thow error if bad adjective given
    if find_only_best:
        rospy.loginfo('Searching for the ' + task.record + ' object...')

    # Do we need to count objects?
    must_count = task.record == 'count'
    if must_count:
        rospy.loginfo('Counting objects that match criteria...')

    # Do we need to only find the first match?
    find_only_first = not find_only_best and not must_count

    if find_only_first:
        result = sweep_from_center(self, task)
    else:
        result = sweep_from_left_to_right(self, task, must_count)

    if result is None:
        if find_only_first:
            self.speech.say("I couldn't find the " + task.name)
        else:
            self.speech.say("I couldn't find any objects.")
        return False

    all_matched_objects = result
    if find_only_first:
        obj = all_matched_objects[0]
        cx, cy = obj.center
        self.head.center_camera(cx, cy)
        self.speech.say("Hey I just saw a " + obj.label + '.')
        self.speech.say("I found the " + task.name)
    elif find_only_best:
        names = [obj.label.lower().replace('_', ' ') for obj in all_matched_objects]
        # Size
        if task.record in ['biggest', 'largest', 'smallest']:
            criteria = [OBJECT_NAME_TO_SIZE[name] for name in names]
        # Weight
        elif task.record in ['heaviest', 'lightest']:
            criteria = [OBJECT_NAME_TO_WEIGHT[name] for name in names]
        # Least
        if task.record in ['smallest', 'lightest']:
            name = names[criteria.index(min(criteria))]
        # Most
        else:
            name = names[criteria.index(max(criteria))]

        txt = name + ' is the ' + task.record + ' object I found.'
        self.speech.say(txt)
        self.recorded_result = txt
    elif must_count:
        all_names = [obj.label for obj in all_matched_objects]
        unique_names = list(set(all_names))
        num_objects = str(len(unique_names))
        txt = "I found " + num_objects + ' ' + task.name
        self.speech.say(txt)
        self.recorded_result = txt

    return True


def sweep_from_center(self, task):
    # Go into start position
    pan = 0
    pan_step = 0.2
    self.head.move(pan=pan,tilt=-0.2,move_time=1)
    # Setup the search
    goals = [-1.0, 1.75, -2.0]
    found = False
    for goal in goals:
        # Begin the search!
        goal_passed = False
        while not goal_passed:
            self.head.move(pan=pan)
            if task.type == 'object':
                result = objects_found(self, task)
            if result is not None:
                return result
            if pan < goal:
                pan += pan_step
                goal_passed = pan >= goal
            else:
                pan -= pan_step
                goal_passed = pan <= goal
    return None


# Calls Darknet, returns objects whose name/category match task.name
def objects_found(self, task):
    # detected_objects = self.darknet_cli.call(GetDetectionsRequest()).objects
    try:
        resp = self.darknet_cli(GetDetectionsRequest())
        detected_objects = resp.objects
        time.sleep(0.1)
        rospy.loginfo('Detected objects:')
        rospy.loginfo(detected_objects)
        # Filter out objects with low confidence scores
        detected_objects = [obj for obj in detected_objects if obj.accuracy > DARKNET_THRESHOLD]
        # Did we find anything?
        if len(detected_objects) > 0:
            found_names = [obj.label.lower().replace('_', ' ') for obj in detected_objects]
            found_categories = [OBJECT_NAME_TO_CATEGORY[found_name] for found_name in found_names]
            matches = [obj for obj in detected_objects if task.name.lower() in found_names + found_categories]
            if matches:
                return matches
        return None
    except rospy.ServiceException, e:
        rospy.logwarn("Lookfor: Darknet Service call failed, exception: {}".format(e))


def sweep_from_left_to_right(self, task, count):
    # Go into start position
    pan = 1.5
    tilt = 0.05
    self.head.move(pan=pan, tilt=tilt, move_time=2)
    # Begin the search!
    all_matched = list()
    end_search = False
    pan_step = 0.2
    while pan > -1.5:
        self.head.move(pan=pan)
        if task.type == 'object':
            accumulate_objects(self, task, count, all_matched)
        pan -= pan_step
        print pan
    return all_matched


def accumulate_objects(self, task, count, all_matched):
    if count:
        matches = objects_found(self, task)
        if matches is not None:
            all_matched += matches
    else:  # find best
        objects = self.darknet_cli.call(GetDetectionsRequest()).objects
        time.sleep(0.1)
        print objects
        # Filter out low confidences
        objects = [obj for obj in objects if obj.accuracy > DARKNET_THRESHOLD]
        all_matched += objects


def accumulate_humans(self, task, req, count, all_matched):
    pass
