#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class Avoid:
    def __init__(self):
        rospy.init_node('Avoid', anonymous=True)
        self.pub = rospy.Publisher('Avoid', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/front/scan', LaserScan,
                                    self.get_vector)  # simulation: front/scan, reality: /scan
        self.rate = rospy.Rate(10)  # 10Hz
        self.x = 0
        self.y = 0
        # repulsive force up to R
        self.R = 2

    def get_vector(self, data):

        self.x = 0
        self.y = 0

        start_angle = np.deg2rad(-60)  # rad (front = 0 rad)
        end_angle = np.deg2rad(60)  # rad (front = 0 rad)

        start = int((start_angle - data.angle_min) / data.angle_increment)
        end = int((end_angle - data.angle_min) / data.angle_increment)

        for i in range(start, end):
            # distance to angle i in meter
            dist_i = data.ranges[i]

            if np.isnan(dist_i):
                dist_i = 0
            if np.isinf(dist_i) or dist_i > self.R:
                dist_i = self.R

            # conversion into cartesian coordinates
            angle = start_angle + data.angle_increment * i
            if dist_i != 0:
                self.x -= (1 / dist_i) * np.cos(angle)
                self.y -= (1 / dist_i) * np.sin(angle)


    def calc_linear_vel(self):
        repulsive_force = (self.x ** 2 + self.y ** 2) ** 0.5
        if repulsive_force == 0:
            return 0
        vel = 10 / repulsive_force

        if abs(vel) > 1.8:
            vel = 1.8 * np.sign(vel)
        return vel


    def calc_angular_vel(self, x, y):
        if x == 0 and y == 0:
            return 0
        angle = np.arctan2(y, x)
        if angle == 0:
            return 0
        vel = angle * 0.4
        if abs(vel) > 1.5:
            vel = 1.5 * np.sign(vel)
        return vel

    def run(self):
        # alle nicht benoetigten Werte auf null setzen
        msg = Twist()
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        while not rospy.is_shutdown():
            msg.angular.z = self.calc_angular_vel(self.x, self.y)
            msg.linear.x = self.calc_linear_vel()

            self.pub.publish(msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Avoid().run()
    except rospy.ROSInterruptException:
        pass
