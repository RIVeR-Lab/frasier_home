#!/usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import WrenchStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import re
import rospy
# from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tmc_msgs.msg import Voice


class Inspection:
    def __init__(self):
        # Params
        self.last_min = 0.0
        self.wrist_wrench = WrenchStamped()
        self.threshold = rospy.get_param('~depth_threshold', 0.25)
        self.googleText = ''
        rospy.init_node('base_test')
        rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, self.wrenchCB)
        # Voice
        self.voice_msg = Voice()
        self.voice_msg.queueing = False
        self.voice_msg.language = Voice.kEnglish
        self.voice_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
        rospy.sleep(1)
        self.pubVoice("Lets start the inspection!")
        self.pubVoice("Please touch my hand after opening the door.")
        # Action client
        self.client = SimpleActionClient('/move_base/move', MoveBaseAction)
        rospy.loginfo("Subscribed to topics")
        # rospy.spin()

    def pubVoice(self, text):
        self.voice_msg.sentence = text
        self.voice_pub.publish(self.voice_msg)
        rospy.sleep(2)

    def wrenchCB(self, data):
        self.wrist_wrench = data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            x = self.wrist_wrench.wrench.force.x
            if x > 17:
                self.gotoPosition()
                break
            rate.sleep()

    def gotoPosition(self):
        self.pubVoice("Thanks for opening the door. I am moving to the inspection location.")
        rospy.loginfo("Creating action client...")
        # Create goal
        rospy.loginfo("Creating MoveBaseGoal...")
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = 6.2
        goal.target_pose.pose.position.y = 2.8
        goal.target_pose.pose.orientation.z = 0.44
        goal.target_pose.pose.orientation.w = 0.87
        # goal.target_pose.pose.position.x = 1
        # goal.target_pose.pose.position.y = 0
        # goal.target_pose.pose.orientation.z = 0.0
        # goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        # Send goal position
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()
        rospy.loginfo("Sending goal...")
        self.client.send_goal(goal)
        rospy.loginfo("Waiting to reach goal...")
        self.client.wait_for_result()
        self.wait()

    def wait(self):
        rospy.loginfo("Waiting at inspection location...")
        self.pubVoice("I am waiting here for inspection. Please tap my hand again when you are done.")
        while True:
            # if re.search(r'\b(exit|quit|leave)\b', self.googleText, re.I):
            if self.wrist_wrench.wrench.force.x > 17:
                rospy.loginfo("Exit command received")
                break
        rospy.loginfo("Waiting completed...")
        self.gotoStart()

    def gotoStart(self):
        rospy.loginfo("Moving to exit location...")
        self.pubVoice("Thanks for inspecting me, goodbye.")
        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = 13.15
        goal.target_pose.pose.position.y = 0.4
        goal.target_pose.pose.orientation.z = -0.34
        goal.target_pose.pose.orientation.w = 0.94
        # goal.target_pose.pose.position.x = 0
        # goal.target_pose.pose.position.y = 1
        # goal.target_pose.pose.orientation.z = 1
        # goal.target_pose.pose.orientation.w = 0.0
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        # Send goal position
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()
        rospy.loginfo("Sending goal...")
        self.client.send_goal(goal)
        rospy.loginfo("Waiting to reach goal...")
        self.client.wait_for_result()

if __name__ == '__main__':
    inspec = Inspection()
    inspec.run()
