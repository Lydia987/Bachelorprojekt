#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np


class Arbiter:

    msgStop = Twist()
    msgAvoid = Twist()
    msgFollowMe = Twist()
    msgWallFollowing = Twist()
    followAngle = 0.0 #grad
    drive = True
    previous_behavoir = "FollowMe"
    counter_wallFollowing = 60
    stop_wallFollwoing = False
    R = 6

    def __init__(self):
        rospy.init_node('Arbiter', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subStop = rospy.Subscriber('Stop', String, self.stop)
        self.subAvoid = rospy.Subscriber('Avoid', Twist, self.avoid)
        self.subFollowMe = rospy.Subscriber('FollowMe', Twist, self.follow)
        self.subFollowMeAngle = rospy.Subscriber('FollowAngle', String, self.set_followAngle)
        self.subEmergencyStop = rospy.Subscriber('btn_stop', Bool, self.shutdown)
        self.sub_wallFollowing = rospy.Subscriber('WallFollowing', Twist, self.wallFollowing)
        self.sub_lidar = rospy.Subscriber('/front/scan', LaserScan, self.set_stop_wallFollowing)  # simulation front/scan, else /scan
        self.rate = rospy.Rate(10)  # 10Hz

    def stop(self, data):
        self.msgStop = Twist()
        self.msgStop.linear.x = 0
        self.msgStop.linear.y = 0
        self.msgStop.linear.z = 0
        self.msgStop.angular.x = 0
        self.msgStop.angular.y = 0
        self.msgStop.angular.z = 0
        if data.data == "stop":
            self.drive = False
        else:
            self.drive = True


    def avoid(self, data):
        self.msgAvoid = data

    def follow(self, data):
        self.msgFollowMe = data

    def set_followAngle(self, data):
        self.followAngle = np.rad2deg(float(data.data))

    def wallFollowing(self, data):
        self.msgWallFollowing = data

    def set_stop_wallFollowing(self, data):
        if self.get_mean_dist(data, self.followAngle - 2, self.followAngle + 2) > 1 or abs(self.msgAvoid.linear.x) <= 0 or abs(self.msgAvoid.angular.z) <= 0:
            self.stop_wallFollwoing = True
        else:
            self.stop_wallFollwoing = False


    def run(self):
        behavior = Twist()
        behavior.linear.y = 0
        behavior.linear.z = 0
        behavior.angular.x = 0
        behavior.angular.y = 0

        while not rospy.is_shutdown():
	    if self.stop_wallFollwoing:
                        self.counter_wallFollowing = 60
            if False:#not self.drive:
                behavior = self.msgStop
                self.previous_behavoir = "Stop"
		print("stop")
            elif abs(self.msgAvoid.linear.x) > 0 or abs(self.msgAvoid.angular.z) > 0:
		print("avoid")
                behavior.linear.x = 0.9 * self.msgAvoid.linear.x + 0.1 * self.msgFollowMe.linear.x
                behavior.angular.z = 0.9 * self.msgAvoid.angular.z + 0.1 * self.msgFollowMe.angular.z
                if self.previous_behavoir == "Avoid":
                    self.counter_wallFollowing -= 1
                self.previous_behavoir = "Avoid"
                if self.counter_wallFollowing == 0:
                    self.previous_behavoir = "WallFollowing"
		    print("wall")
                    behavior = self.msgWallFollowing
            else:
                behavior = self.msgFollowMe
                self.previous_behavoir = "FollowMe"
		print("follow")

            self.pub.publish(behavior)
            rospy.on_shutdown(self.shutdown)
            self.rate.sleep()


    def get_mean_dist(self, data, min_angle, max_angle):
	
        sum_dist = 0
        x = 0
        y = 0
        angle = 0

        start_angle = np.deg2rad(min_angle)  # rad (front = 0 rad)
        end_angle = np.deg2rad(max_angle)  # rad (front = 0 rad)

	if start_angle < data.angle_min:
		start_angle = data.angle_min
	if end_angle > data.angle_max:
		end_angle = data.angle_max

        start_range = int((start_angle - data.angle_min) / data.angle_increment)
        end_range = int(len(data.ranges) - ((data.angle_max - end_angle) / data.angle_increment))

	if min_angle == max_angle:
		return data.ranges[start_range]

        for i in range(start_range, end_range):
            # aktuelle Distanz zum Winkel i
            d = data.ranges[i]
            if (i % 2 == 0):
                # Grenzfälle ausschließen
                if np.isnan(d):
                    d = 0
                elif np.isinf(d) or d > self.R:
                    d = self.R

                # Umrechnung in kartesische Koordinaten
                angle = start_angle + data.angle_increment * i
                if d != 0:
                    x += (1 / d) * np.cos(angle)
                    y += (1 / d) * np.sin(angle)
                    sum_dist += np.sqrt(x ** 2 + y ** 2)

        mean_dist = sum_dist / ((end_range - start_range)/ data.angle_increment)

        return mean_dist


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
