#!/usr/bin/env python

# ROS
import rospy
import actionlib

# HSR
from tmc_msgs.msg import Voice
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from hsr_constants import *
from tmc_msgs.msg import *

# Constants
FULL_PAN = 1.16  # Minimum change required to pan completely away
FULL_TILT = 0.9  # Minimum change required to tilt completely away
IM_WIDTH = 640
IM_HEIGHT = 480


class SpeechClient(object):
    def __init__(self):
        # Voice Msg
        self.voice_pub = rospy.Publisher(SPEECH_TOPIC, Voice, queue_size=10)
        self.speech_cli = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        self.speech_cli.wait_for_server(rospy.Duration(3))
        self.voice_msg = Voice()
        self.voice_msg.language = Voice.kEnglish
        self.voice_msg.queueing = True
        self.voice_msg.interrupting = True
        self.voice_msg.sentence = ''

    def setSentence(self, text):
        self.voice_msg.sentence = text

    def say(self, text, blocking=True):
        self.setSentence(text)
        goal = TalkRequestGoal()
        goal.data = self.voice_msg
        self.speech_cli.send_goal(goal)
        if blocking:
            self.speech_cli.wait_for_result()


class ArmClient(object):
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient(ARM_CLIENT_TOPIC, FollowJointTrajectoryAction)
        self.arm_client.wait_for_server(rospy.Duration(3))
        self.arm_goal = FollowJointTrajectoryGoal()
        self.arm_goal.trajectory.joint_names = ARM_JOINTS
        self.p = JointTrajectoryPoint()

    def send_goal(self, arm_goal=None):
        if arm_goal is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

    def spr_start(self):
        self.p.positions = [0.75, -2.5, 0, 1.0, 0]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.arm_goal.trajectory.points = [self.p]
        self.send_goal()

    def hmc_start(self):
        self.p.positions = [0.5, -2.5, 0, 1.0, 0]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.arm_goal.trajectory.points = [self.p]
        self.send_goal()


class HeadClient(object):
    def __init__(self):
        self.head_client = actionlib.SimpleActionClient(HEAD_CLIENT_TOPIC, FollowJointTrajectoryAction)
        self.head_state = rospy.Subscriber(HEAD_STATE_TOPIC, JointTrajectoryControllerState, self.update_state)
        self.head_client.wait_for_server(rospy.Duration(3))
        self.head_goal = FollowJointTrajectoryGoal()
        self.head_goal.trajectory.joint_names = HEAD_JOINTS
        self.p = JointTrajectoryPoint()
        self.current_tilt = 0.0
        self.current_pan = 0.0

    def update_state(self, msg):
        self.current_tilt, self.current_pan = msg.actual.positions

    def send_goal(self, head_goal=None, wait=True):
        if head_goal is None:
            self.head_client.send_goal(self.head_goal)
        else:
            self.head_client.send_goal(head_goal)
        if wait:
            self.head_client.wait_for_result()

    # Pan: [-3.84,1.75] [right,left]
    # Tilt: [-1.57,0.52] [down,up]
    def move(self, pan=None, tilt=None, velocities=[0, 0], move_time=1):
        if pan is None:
            pan = self.current_pan
        if tilt is None:
            tilt = self.current_tilt
        self.p.positions = [pan, tilt]
        self.p.velocities = velocities
        self.p.time_from_start = rospy.Time(move_time)
        self.head_goal.trajectory.points = [self.p]
        self.send_goal()

    def center_camera(self, x, y, x_goal=0.5, y_goal=0.5, move_time=0.8):
        if x_goal is not None:
            if x > 1:
                x = float(x) / float(IM_WIDTH)
            rospy.loginfo('Start x: ' + str(x))
            pan_step = (x_goal - x) * FULL_PAN
        else:
            pan_step = 0
        if y_goal is not None:
            if y > 1:
                y = float(y) / float(IM_HEIGHT)
            rospy.loginfo('Start y: ' + str(y))
            tilt_step = (y_goal - y) * FULL_TILT
        else:
            tilt_step = 0

        pan = self.current_pan + pan_step
        tilt = self.current_tilt + tilt_step

        self.move(pan=pan, tilt=tilt, move_time=move_time)

    def get_current_position(self):
        return self.current_pan, self.current_tilt

    def spr_start(self):
        self.p.positions = [0.0, 0.0]
        self.p.velocities = [0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.head_goal.trajectory.points = [self.p]
        self.send_goal()

    def hmc_start(self):
        self.p.positions = [0.0, 0.0]
        self.p.velocities = [0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.head_goal.trajectory.points = [self.p]
        self.send_goal()

    def set_point(self, point=None):
        if point is None:
            self.head_goal.trajectory.points = [self.p]
        else:
            self.head_goal.trajectory.points = point

    def set_position(self, positions):
        self.p.positions = positions

    def set_velocity(self, velocities):
        self.p.velocities = velocities

    def set_time(self, time):
        self.p.time_from_start = rospy.Time(time)
