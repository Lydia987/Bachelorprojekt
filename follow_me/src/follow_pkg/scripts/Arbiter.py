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
    followAngle = 0.0  # grad
    drive = True
    previous_behavior = "FollowMe"
    counter_wallFollowing = 30
    stop_wall_following = False
    R = 2.6

    def __init__(self):
        rospy.init_node('Arbiter', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subStop = rospy.Subscriber('Stop', String, self.stop)
        self.subAvoid = rospy.Subscriber('Avoid', Twist, self.avoid)
        self.subFollowMe = rospy.Subscriber('FollowMe', Twist, self.follow)
        self.subFollowMeAngle = rospy.Subscriber('FollowAngle', String, self.set_follow_angle)
        self.subEmergencyStop = rospy.Subscriber('btn_stop', Bool, self.shutdown)
        self.sub_wallFollowing = rospy.Subscriber('WallFollowing', Twist, self.wall_following)
        self.sub_lidar = rospy.Subscriber('/front/scan', LaserScan,
                                          self.set_stop_wall_following)  # simulation: front/scan, reality: /scan
        self.rate = rospy.Rate(10)  # Hz

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

    def set_follow_angle(self, data):
        self.followAngle = np.rad2deg(float(data.data))

    def wall_following(self, data):
        self.msgWallFollowing = data

    def set_stop_wall_following(self, data):
        if self.get_mean_dist(data, self.followAngle - 4,
                              self.followAngle + 4) > 2.5 or (
                self.get_mean_dist(data, -70, -40) > 2 and self.get_mean_dist(data, 40, 70) > 2.5):
            self.stop_wall_following = True
        else:
            self.stop_wall_following = False

    def run(self):
        behavior = Twist()
        behavior.linear.y = 0
        behavior.linear.z = 0
        behavior.angular.x = 0
        behavior.angular.y = 0

        while not rospy.is_shutdown():
            if self.stop_wall_following and self.previous_behavior == "WallFollowing":
                self.counter_wallFollowing = 30
            if False:  # not self.drive:
                behavior = self.msgStop
                self.previous_behavior = "Stop"
                print("stop")
            elif abs(self.msgAvoid.linear.x) > 0 or abs(self.msgAvoid.angular.z) > 0:
                print("avoid")
                behavior.linear.x = 0.9 * self.msgAvoid.linear.x + 0.1 * self.msgFollowMe.linear.x
                behavior.angular.z = 0.9 * self.msgAvoid.angular.z + 0.1 * self.msgFollowMe.angular.z
                if self.previous_behavior == "Avoid":
                    self.counter_wallFollowing -= 1
                self.previous_behavior = "Avoid"
                if self.counter_wallFollowing == 0:
                    self.previous_behavior = "WallFollowing"
                    print("wall")
                    behavior = self.msgWallFollowing
            else:
                behavior = self.msgFollowMe
                self.previous_behavior = "FollowMe"
                print("follow")

            self.pub.publish(behavior)
            rospy.on_shutdown(self.shutdown)
            self.rate.sleep()

    def get_mean_dist(self, data, min_angle, max_angle):

        sum_dist = 0.0
        start_angle = np.deg2rad(min_angle)  # rad (front = 0 rad)
        end_angle = np.deg2rad(max_angle)  # rad (front = 0 rad)

        if start_angle < data.angle_min:
            start_angle = data.angle_min
        if end_angle > data.angle_max:
            end_angle = data.angle_max

        start = int((start_angle - data.angle_min) / data.angle_increment)
        end = int((end_angle - data.angle_min) / data.angle_increment)

        if min_angle == max_angle:
            return data.ranges[start]

        for i in range(start, end):
            dist_i = data.ranges[i]
            if np.isnan(dist_i):
                dist_i = 0.0
            if np.isinf(dist_i) or self.R:
                dist_i = self.R

            sum_dist += dist_i

        mean_dist = sum_dist / (end - start)

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
