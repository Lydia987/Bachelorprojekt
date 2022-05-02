#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import decimal as d


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

        self.counter = 20
        self.msg = "drive"
        self.vel = 0
        self.rate = rospy.Rate(10)
        self.actual_pos = [d.Decimal(0), d.Decimal(0)]  # [latitude,longitude]
        self.old_pos = [d.Decimal(0), d.Decimal(0)]  # [latitude,longitude]

    # Stops the robot if something is nearer than 0,4 meter
    def laser_callback(self, scan_msg):
        for i in scan_msg.ranges:
            if (i <= 0.3 or (i <= 0.4 and self.msg == "stop")) and not np.isinf(i) and not np.isnan(i):
                self.msg = "stop"
                break
            else:
                self.msg = "drive"

    # the position of the robot
    def set_pos(self, data):
        self.actual_pos[0] = d.Decimal(data.latitude)
        self.actual_pos[1] = d.Decimal(data.longitude)
        print("pos", data.latitude, data.longitude)

    # the velocity of the robot
    def set_velocity(self, data):
        self.vel = data.linear.x

    def run(self):
        while not rospy.is_shutdown():
            dist = d.Decimal(d.Decimal(np.sqrt(d.Decimal(d.Decimal(d.Decimal(self.old_pos[0] - self.actual_pos[0]) * d.Decimal(111.3)) ** 2) + d.Decimal(d.Decimal(d.Decimal(self.old_pos[1] - self.actual_pos[1]) * d.Decimal(71.5)) ** 2))))
            print(dist)
            if dist < 0.0001 and not(self.vel == 0) and self.counter == 0:
                self.msg = "festgefahren"

            if self.counter == 0:
                self.old_pos = self.actual_pos
                self.counter = 20
            else:
                self.counter -= 1

            self.stop_pub.publish(self.msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Stop().run()
    except rospy.ROSInterruptException:
        pass
