#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from tmc_msgs.msg import Voice
from geometry_msgs.msg import WrenchStamped

class wrist:
    def __init__(self):
        rospy.init_node('frasier_inspection')
        self.wrist_wrench = WrenchStamped()
        try:
            rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, self.callback)
            self.talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

            # rospy.spin()

        except rospy.ROSInterruptException:
            pass

    def callback(self, data):
        self.wrist_wrench = data
        
    def run(self):
        start = Voice()
        start.language = Voice.kEnglish
        start.queueing = False
        # start.interrupting = False
        start.sentence = "Please touch my hand after you open the door"
        self.talk_pub.publish(start)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            x = self.wrist_wrench.wrench.force.x
            rospy.loginfo('touch my hand!')
            if x > 17:
                break
            rate.sleep()


if __name__ == '__main__':
    run = wrist()
    run.run()
