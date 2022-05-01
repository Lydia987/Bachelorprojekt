#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import numpy as np


class Stop:
    def __init__(self):
        rospy.init_node('Stop', anonymous=True)

        # reality
        # rospy.Subscriber('/scan', LaserScan, laser_callback)

        # simulation
        rospy.Subscriber('/front/scan', LaserScan, self.laser_callback)

        # always
        self.stop_pub = rospy.Publisher('Stop', String, queue_size=10)
        self.sub_roboter_gps = rospy.Subscriber('/navsat/fix', NavSatFix, self.set_pos)
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.set_velocity)

        self.counter = 2500
        self.msg = "drive"
        self.vel = 0
        self.rate = rospy.Rate(10)
        self.actual_pos = [0.0, 0.0]  # [latitude,longitude]
        self.old_pos = [0.0, 0.0]  # [latitude,longitude]

    # Stops the robot if something is nearer than 0,4 meter
    def laser_callback(self, scan_msg):
        for i in scan_msg.ranges:
            if (i <= 0.4 or (i <= 0.5 and self.msg == "stop")) and not np.isinf(i) and not np.isnan(i):
                self.msg = "stop"
                break
            else:
                self.msg = "drive"

    # the position of the robot
    def set_pos(self, data):
        self.actual_pos[0] = data.latitude
        self.actual_pos[1] = data.longitude

    # the velocity of the robot
    def set_velocity(self, data):
        self.vel = data.linear.x

    def run(self):
        while not rospy.is_shutdown():
            dist = abs(self.old_pos[0] - self.actual_pos[0]) + (abs(self.old_pos[1] - self.actual_pos[1]))
            if dist == 0 and not(self.vel == 0):
                self.msg = "festgefahren"

            if self.counter == 0:
                self.old_pos = self.actual_pos
                self.counter = 2500
            else:
                self.counter -= 1

            self.stop_pub.publish(self.msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Stop().run()
    except rospy.ROSInterruptException:
        pass
