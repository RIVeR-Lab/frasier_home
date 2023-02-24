# ROS
import rospy, rospkg

# Python
import numpy as np
from frasier_person_description.srv import DescribePeopleRequest
import sys
pkg_dir = rospkg.RosPack().get_path('frasier_person_description')
sys.path.append(pkg_dir + '/src/deepfashion')
import color_detectv2 as color_detect

BODY_PARTS = ["Nose","Neck","RShoulder","RElbow","RWrist","LShoulder",
			 "LElbow","LWrist","RHip","RKnee","RAnkle","LHip","LKnee",
			 "LAnkle","REye","LEye","REar","LEar"]
BODY_PARTS_TO_ID = {x:ind for ind,x in enumerate(BODY_PARTS)}

FULL_PAN = 1.16  # Minimum change required to pan completely away
FULL_TILT = 0.9

def center_camera_on_all_people(self,move_time=0.5):
    pts = get_center_of_all_poses(self)
    if pts is not None:
        self.head.center_camera(pts[0],pts[1])

def get_center_of_all_poses(self):
    pose_msgs = get_just_poses(self)
    if pose_msgs is None:
        return None
	x_points = [x.part_x for x in pose_msgs if x.part_x != -1]
	y_points = [x.part_y for x in pose_msgs if x.part_y != -1]
	x_mid = np.ptp(x_points)+min(x_points)
	y_mid = np.ptp(y_points)+min(y_points)

	return x_mid, y_mid

def get_just_poses(self):
    req = DescribePeopleRequest(gender=False,
                                age=False,
                                emotion=False,
                                upper_fashion=False,
                                bottom_fashion=False)
    descriptions = self.describe_cli.call(req)
    if descriptions.num_people == 0:
        return None
    pose_msgs = [descriptions.poses[x] for x in xrange(descriptions.num_people)]

    return pose_msgs

def generate_descriptions_summary(descriptions):	
    txts = ['<img src="summary.jpg" alt="User Image" height=100%><br><h1>Detected Poses</h1><br>']

    for x in xrange(descriptions.num_people):
        uc = color_detect.closest_colour(descriptions.upper_hexcodes[x])
        bc = color_detect.closest_colour(descriptions.bottom_hexcodes[x])

        txt = '<img src="'+str(x)+'.jpg" alt="User Image" height=100%>'
        txt += '<h1>Gender: '+descriptions.genders[x]
        txt += '<br>Age: '+descriptions.ages[x]
        txt += '<br>Emotion: '+descriptions.emotions[x]
        txt += '<br>Upper clothing: '+uc+' '+descriptions.upper_fashions[x]
        txt += '<br>Bottom clothing: '+bc+' '+descriptions.bottom_fashions[x]+'</h1>'

        txts.append(txt)
    return txts



def generate_descriptions_text(descriptions):
    sentences = list()

    for x in xrange(descriptions.num_people):
        txt = ''
        if descriptions.genders[x] == '':
            txt += 'a person '
        else:
            txt += 'a '+descriptions.genders[x]+' '
        if descriptions.ages[x] != '':
            lower_age,upper_age = descriptions.ages[x].split(',')
            txt += 'between the age of '+lower_age+' and '+upper_age+' '
        if descriptions.emotions[x] != '':
            txt += 'with a '+descriptions.emotions[x]+' expression '
        if descriptions.upper_fashions[x] != '':
            clothing = descriptions.upper_fashions[x]
            color = color_detect.closest_colour(descriptions.upper_hexcodes[x])
            fashion = color+' '+clothing
            txt += 'wearing a '+fashion+' '
            if descriptions.bottom_fashions[x] != '':
                clothing = descriptions.bottom_fashions[x]
                color = color_detect.closest_colour(descriptions.bottom_hexcodes[x])
                fashion = color+' '+clothing
                txt += 'and a '+fashion+' '
        elif descriptions.bottom_fashions[x] != '':
            clothing = descriptions.bottom_fashions[x]
            color = color_detect.closest_colour(descriptions.bottom_hexcodes[x])
            fashion = color+' '+clothing
            txt += 'wearing '+fashion+' '
        sentences.append(txt)
    all_descriptions = 'I see '+'and '.join(sentences)
    print all_descriptions
    return all_descriptions

