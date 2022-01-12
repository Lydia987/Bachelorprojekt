#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class Arbiter:
    msgStop = Twist()
    msgAvoid = Twist()
    msgFollowMe = Twist()

    def __init__(self):
        rospy.init_node('Arbiter', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subStop = rospy.Subscriber('Stop', String, self.stop)
        self.subAvoid = rospy.Subscriber('Avoid', Twist, self.avoid)
        self.subFollowMe = rospy.Subscriber('FollowMe', Twist, self.follow)
        self.rate = rospy.Rate(10)  # 10Hz

    def stop(self, data):
        self.msgStop = data.data

    def avoid(self, data):
        self.msgAvoid = data

    def follow(self, data):
        self.msgFollowMe = data

    def run(self):
        msg = Twist()
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0

        while not rospy.is_shutdown():
            if self.msgStop == "stop":
                self.shutdown()
            elif abs(self.msgAvoid.linear.x) > 0 and abs(self.msgAvoid.angular.z) > 0:
                msg.linear.x = (0.9 * self.msgAvoid.linear.x + 0.1 * self.msgFollowMe.linear.x)/3
                msg.angular.z = (0.9 * self.msgAvoid.angular.z + 0.1 * self.msgFollowMe.angular.z)/3
	    else:
		msg.linear.x = self.msgFollowMe.linear.x
                msg.angular.z = self.msgFollowMe.angular.z
            self.pub.publish(msg)
            rospy.on_shutdown(self.shutdown)
            self.rate.sleep()

    # stop robot when node is stopped
    def shutdown(self):
        Twist().linear.x = 0
        Twist().linear.y = 0
        Twist().linear.z = 0
        Twist().angular.x = 0
        Twist().angular.y = 0
        Twist().angular.z = 0

        self.pub.publish(Twist())


if __name__ == '__main__':
    try:
        Arbiter().run()
    except rospy.ROSInterruptException:
        pass
