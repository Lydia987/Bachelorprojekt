#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class WallFollowing:
    def __init__(self):
        rospy.init_node('WallFollowing', anonymous=True)
        self.pub = rospy.Publisher('WallFollowing', Twist, queue_size=10) # eigentlich WallFollowing, nur zum alleinigen Test /cmd_vel
        self.sub = rospy.Subscriber('/front/scan', LaserScan,
                                    self.getVector)  # für Simulation /front/scan für Realität /scan
        self.rate = rospy.Rate(10)  # 10Hz
        self.R = 1


    # Einlesen der Sensordaten und Berechnen von x und y
    def getVector(self, data):

        msg = Twist()
        msg.linear.x = 1
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0

        right_mean_dist = self.get_mean_dist(data,-45,-10)
        left_mean_dist = self.get_mean_dist(data, 10, 45)

        if left_mean_dist >= right_mean_dist:
            # Follow right wall
		msg.angular.z = -14*(0.15-right_mean_dist)
        else:
            # Follow left wall
		msg.angular.z = 14*(0.15-left_mean_dist)

	    print("msg",msg)
        self.pub.publish(msg)

    def get_mean_dist(self, data, min_angle, max_angle):
        sum_dist = 0
        x = 0
        y = 0
        angle = 0

        start_angle = np.deg2rad(min_angle)  # rad (front = 0 rad)
        end_angle = np.deg2rad(max_angle)  # rad (front = 0 rad)

        start_range = int((start_angle - data.angle_min) / data.angle_increment)
        end_range = int(len(data.ranges) - ((data.angle_max - end_angle) / data.angle_increment))

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


    def run(self):
        rospy.Rate(10) #Hz
        rospy.spin()


if __name__ == '__main__':
    try:
        WallFollowing().run()
    except rospy.ROSInterruptException:
        pass
