#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


# calculates the twist message to follow the wall
class WallFollowing:
    desired_dist = 1.1  # desired distance to the wall
    correction_strength = 0.9
    R = 2.5  # repulsive force up to R in meter
    follow_right = True

    def __init__(self):
        rospy.init_node('WallFollowing', anonymous=True)

        # reality
        # self.sub = rospy.Subscriber('/scan', LaserScan, self.get_vector)

        # simulation
        self.sub = rospy.Subscriber('/front/scan', LaserScan, self.get_vector)

        # always
        self.pub = rospy.Publisher('WallFollowing', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Hz

    # calculates the vector to follow the wall
    def get_vector(self, data):
        msg = Twist()
        msg.linear.x = 1

        left_mean_dist = self.get_mean_dist(data, -70, -30)
        front_mean_dist = self.get_mean_dist(data, -30, 30)
        right_mean_dist = self.get_mean_dist(data, 30, 70)

        if left_mean_dist >= right_mean_dist:
            # Follow right wall
            self.follow_right = True
            msg.angular.z = (-1.0) * self.correction_strength * (self.desired_dist - right_mean_dist)
        else:
            # Follow left wall
            self.follow_right = False
            msg.angular.z = self.correction_strength * (self.desired_dist - left_mean_dist)
        if front_mean_dist < self.desired_dist + 0.4:
            if self.follow_right:
                # turn left:
                msg.angular.z = -2
            else:
                # turn right:
                msg.angular.z = 2

        self.pub.publish(msg)

    # calculates the mean distance to obstacles between the min_angle and max_angle
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
            if np.isinf(dist_i) or dist_i > self.R:
                dist_i = self.R

            sum_dist += dist_i

        mean_dist = sum_dist / (end - start)
        return mean_dist

    def run(self):
        rospy.Rate(10)
        rospy.spin()


if __name__ == '__main__':
    try:
        WallFollowing().run()
    except rospy.ROSInterruptException:
        pass
