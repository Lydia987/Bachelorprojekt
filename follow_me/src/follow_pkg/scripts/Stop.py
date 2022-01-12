#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

msg = "drive"
# Stops the robot if something is nearer then 0,8 meter #


# Einlesen der Sensordaten und Stoppen bei unterschreiten eines bestimmen Wertes
def laser_callback(scan_msg):
    global msg
    for i in scan_msg.ranges:
        if i <= 0.2 and not np.isinf(i) and not np.isnan(i):  # Meter
            msg = "stop"
        #else:
	#    msg = "drive"
        
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('Stop', anonymous=True)
    pub = rospy.Publisher('Stop', String, queue_size=10)
    rospy.Subscriber('/front/scan', LaserScan, laser_callback)  # für Simulation /front/scan sonst /scan
    rospy.Rate(10)  # 10Hz
    rospy.spin()

