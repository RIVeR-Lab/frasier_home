import numpy as np
import time
from frasier_person_description.srv import DescribePeopleRequest
import lookfor
import rospy
import math

BODY_PARTS = ["Nose","Neck","RShoulder","RElbow","RWrist","LShoulder",
			 "LElbow","LWrist","RHip","RKnee","RAnkle","LHip","LKnee",
			 "LAnkle","REye","LEye","REar","LEar"]
BODY_PARTS_TO_ID = {x:ind for ind,x in enumerate(BODY_PARTS)}
RAISE_RIGHT_ARM = 'raising their right arm'
RAISE_LEFT_ARM = 'raising their left arm'
WAVING = 'waving'
POINTING_LEFT = 'pointing to the left'
POINTING_RIGHT = 'pointing to the right'

FULL_PAN = 1.16  # Minimum change required to pan completely away
FULL_TILT = 0.9

def is_same_point(x1,y1,x2,y2):
	return abs(x1-x2)<0.15 and abs(y1-y2)<0.15

def get_centered_pan(current_pan,x):
    pan_step = (0.5 - x) * FULL_PAN
    pan = current_pan + pan_step
    return pan

def get_centered_tilt(current_tilt,y):
    tilt_step = (0.5 - y) * FULL_TILT
    tilt = current_tilt + tilt_step
    return tilt

def pan_to_pose(self,pose_msg):
	cx,cy = get_pose_center(pose_msg)
	self.head.center_camera(cx,cy,y_goal=None)

def get_pose_center(pose_msg):
	part_x = pose_msg.part_x
	part_y = pose_msg.part_y
	filtered_part_x = [x for x in part_x if x != -1]
	filtered_part_y = [x for x in part_y if x != -1]
	if len(filtered_part_x) == 1:
		return filtered_part_x[0],filtered_part_y[0]
	cx = np.ptp(filtered_part_x)/2.0 + min(filtered_part_x)
	cy = np.ptp(filtered_part_y)/2.0 + min(filtered_part_y)
	return cx,cy

def get_face_center(pose_msg):
	cx = -1
	cy = -1
	found = False

	part_x = pose_msg.part_x
	part_y = pose_msg.part_y

	xdict = {BODY_PARTS[ind]:x for ind,x in enumerate(part_x)}
	ydict = {BODY_PARTS[ind]:x for ind,x in enumerate(part_y)}

	if xdict["Nose"] != -1:
		cx = xdict["Nose"]
		cy = ydict["Nose"]
		found = True
	else:
		filtered_part_x = [xdict[key] \
							for key in ["REye","LEye"] \
							if xdict[key] != -1]
		filtered_part_y = [ydict[key] \
							for key in ["REye","LEye"] \
							if ydict[key] != -1]
		if len(filtered_part_x) == 1:
			cx = filtered_part_x[0]
			cy = filtered_part_y[0]
			found = True
		elif len(filtered_part_x) > 1:
			cx = np.ptp(filtered_part_x)/2.0 + min(filtered_part_x)
			cy = np.ptp(filtered_part_y)/2.0 + min(filtered_part_y)
			found = True
	return cx,cy,found

def get_centermost_pose_face(self):
	pose_msgs = get_just_poses(self)
	if pose_msgs is None:
		return None
	centermost_pose = get_centermost_pose(pose_msgs)
	cx,cy,found = get_face_center(centermost_pose)
	if not found:
		return None
	return cx,cy

# Index False: Given an array of pos_msgs, return the one closest to the center
# Index True: Given an array of descriptions, return the index of the centermost one
def get_centermost_pose(pose_msgs,index=False):
	if index:
		pose_msgs = [pose_msgs.poses[x] for x in xrange(pose_msgs.num_people)]
	min_x_dist = None
	center_index = None
	for ind,pose_msg in enumerate(pose_msgs):
		x_pos,_ = get_pose_center(pose_msg)
		x_dist = abs(x_pos-0.5)
		if min_x_dist is None or x_dist < min_x_dist:
			center_pose_msg = pose_msg 
			min_x_dist = x_dist
			center_index = ind
	if index:
		return center_index
	else:
		return center_pose_msg

def count_points(pose_msg,segment):
	if segment == 'face':
		pts_of_interest = ['Nose','REye','LEye']
	elif segment == 'upper':
		pts_of_interest = ['RShoulder','RElbow','RWrist','LShoulder',
							'LElbow','LWrist','RHip','LHip']
	else:
		pts_of_interest = ['RHip','RKnee','RAnkle','LHip','LKnee','LAnkle']
	
	count = 0
	part_x = pose_msg.part_x
	for ind,part_x in enumerate(part_x):
		if part_x != -1 and BODY_PARTS[ind] in pts_of_interest:
			count += 1
	return count

# We should get a stream of poses so that we can also detect waving
def get_gesture_of_centermost(self):
	end_time = time.time()+1.5
	center_pose_msgs = list()
	while time.time() < end_time:
		timer = time.time()
		pose_msgs = get_just_poses(self)
		center_pose_msg = get_centermost_pose(pose_msgs)
		center_pose_msgs.append(center_pose_msg)

	left_wrist_x_points = [pose_msg.part_x[BODY_PARTS_TO_ID['LWrist']] \
							for pose_msg in center_pose_msgs]
	right_wrist_x_points = [pose_msg.part_x[BODY_PARTS_TO_ID['RWrist']] \
							for pose_msg in center_pose_msgs]

	if is_waving(left_wrist_x_points) or is_waving(right_wrist_x_points):
		return WAVING

	left_wrist_y_points = [pose_msg.part_y[BODY_PARTS_TO_ID['LWrist']] \
							for pose_msg in center_pose_msgs]
	right_wrist_y_points = [pose_msg.part_y[BODY_PARTS_TO_ID['RWrist']] \
							for pose_msg in center_pose_msgs]

	left_shoulder_y_points = [pose_msg.part_y[BODY_PARTS_TO_ID['LShoulder']] \
							  for pose_msg in center_pose_msgs]
	right_shoulder_y_points = [pose_msg.part_y[BODY_PARTS_TO_ID['RShoulder']] \
							   for pose_msg in center_pose_msgs]
	
	rospy.loginfo('LWrist')
	rospy.loginfo(left_wrist_y_points)
	rospy.loginfo('RWrist')
	rospy.loginfo(right_wrist_y_points)
	rospy.loginfo('LShoulder')
	rospy.loginfo(left_shoulder_y_points)
	rospy.loginfo('RShoulder')
	rospy.loginfo(right_shoulder_y_points)


	if is_raising(left_wrist_y_points,left_shoulder_y_points):
		return RAISE_LEFT_ARM
	if is_raising(right_wrist_y_points,right_shoulder_y_points):
		return RAISE_RIGHT_ARM
	if is_pointing_right(center_pose_msgs[-1]):
		return POINTING_RIGHT
	if is_pointing_left(center_pose_msgs[-1]):
		return POINTING_LEFT

	return 'not doing any gesture.'

def is_pointing_right(pose_msg):
	left_angle,right_angle,left_wrist_x,right_wrist_x = get_wrists_and_angles(pose_msg)

	return abs(left_angle)<15 or abs(left_angle+160)<15 or abs(right_angle)<15 or abs(right_angle+160)<15

def is_pointing_left(pose_msg):
	left_angle,right_angle,left_wrist_x,right_wrist_x = get_wrists_and_angles(pose_msg)

	return abs(left_angle-180)<30 or abs(right_angle-180)<20

def is_raising(wrist_y_points,shoulder_y_points):
	count = 0
	for ind,x in enumerate(wrist_y_points):
		if x != -1 and x < shoulder_y_points[ind]:
			count += 1
	percent_raised = float(count)/float(len(wrist_y_points))
	return percent_raised > 0.6

def is_waving(wrist_x_points):
	# Get local extremas
	wrist_x_extrema = get_extrema(wrist_x_points)	
	peak_to_peaks = [x for i,x in enumerate(wrist_x_extrema) if i>0 and abs(x-wrist_x_extrema[i-1])>0.02]

	return len(peak_to_peaks) > 4

def get_extrema(x_points):
	# Filter out -1s
	x_points = [x for x in x_points if x != -1]
	# Only keep local extrema
	x_points = [x for ind,x in enumerate(x_points) \
				 if 0 < ind < len(x_points)-1 \
				 and (x_points[ind-1] < x < x_points[ind+1] \
				 	  or x_points[ind-1] > x > x_points[ind+1])]
	return x_points

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
    # pose_msgs = remove_tiny_people(pose_msgs)
    return pose_msgs

def remove_tiny_people(pose_msgs):
	big_ppl = [x for x in pose_msgs if get_height(x)<0.2]
	return big_ppl

def get_height(pose_msg):
	part_y = pose_msg.part_y
	height = abs(np.ptp(part_y))
	return height

def get_width(pose_msg):
	part_x = pose_msg.part_x
	width = abs(np.ptp(part_x))
	return width

def is_lying_down(pose_msg):
	pass

def try_find_face(self):
	tilt = self.head.current_tilt
	face_coors = None
	while face_coors is None or tilt < 0.4:
		tilt += 0.2
		self.head.move(tilt=tilt)
		face_coors = get_centermost_pose_face(self)
		if face_coors is not None:
			self.head.center_camera(face_coors[0],face_coors[1])
			return True
	return False












def is_correct(self):
	while 1:
		if check_center_arms_no(self):
			return False
		if check_center_arms_yes(self):
			return True

def check_center_arms_no(self):
	pose_msgs = get_just_poses(self)
	if pose_msgs is None:
		return False
	pose_msg = get_centermost_pose(pose_msgs)

	return arms_no(get_wrists_and_angles(pose_msg))

def check_center_arms_yes(self):
	pose_msgs = get_just_poses(self)
	if pose_msgs is None:
		return False
	pose_msg = get_centermost_pose(pose_msgs)

	return arms_yes(get_wrists_and_angles(pose_msg))

def get_wrists_and_angles(pose_msg):
	left_wrist_x  = pose_msg.part_x[BODY_PARTS_TO_ID['LWrist']]
	left_wrist_y  = pose_msg.part_y[BODY_PARTS_TO_ID['LWrist']]

	left_elbow_x = pose_msg.part_x[BODY_PARTS_TO_ID['LElbow']]
	left_elbow_y = pose_msg.part_y[BODY_PARTS_TO_ID['LElbow']]

	right_wrist_x  = pose_msg.part_x[BODY_PARTS_TO_ID['RWrist']]
	right_wrist_y  = pose_msg.part_y[BODY_PARTS_TO_ID['RWrist']]

	right_elbow_x = pose_msg.part_x[BODY_PARTS_TO_ID['RElbow']]
	right_elbow_y = pose_msg.part_y[BODY_PARTS_TO_ID['RElbow']]

	left_angle = arm_angle(left_wrist_x,left_wrist_y,left_elbow_x,left_elbow_y)
	right_angle = arm_angle(right_wrist_x,right_wrist_y,right_elbow_x,right_elbow_y)

	return left_angle,right_angle,left_wrist_x,right_wrist_x

def arms_no(left_angle,right_angle,left_wrist_x,right_wrist_x):
	# Distance of upper-lower
	return abs(left_angle-135)<20 \
			and abs(right_angle-45)<20 \
			and left_wrist_x<right_wrist_x

def arms_yes(left_angle,right_angle,left_wrist_x,right_wrist_x):
	# Distance of upper-lower
	return abs(left_angle-0)<20 \
			and (abs(right_angle-180)<10 or abs(right_angle+180)<10) \
			and left_wrist_x>right_wrist_x

def arm_angle(wrist_x,wrist_y,elbow_x,elbow_y):
	forearm_x_dist = wrist_x-elbow_x
	forearm_y_dist = elbow_y-wrist_y
	return math.degrees(math.atan2(forearm_y_dist,forearm_x_dist))