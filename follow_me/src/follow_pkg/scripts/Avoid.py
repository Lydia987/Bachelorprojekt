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
		self.sub = rospy.Subscriber('/scan', LaserScan, self.getVector) # für Simulation /front/scan
		self.rate = rospy.Rate(10) #10Hz
		# x- und y-Komponenten des gemessenen Vektors
		self.x_vector = 0
		self.y_vector = 0
		# bis R wird eine abstoßende Kraft erzeugt
		self.R = 2


	#Einlesen der Sensordaten und Berechnen von x und y
	def getVector(self, data):
		
		self.x_vector = 0
		self.y_vector = 0

		start_angle = np.deg2rad(-60) #rad (front = 0 rad)
		end_angle = np.deg2rad(60) #rad (front = 0 rad)
		
		start_range = int((start_angle - data.angle_min) / data.angle_increment)
		end_range = int(len(data.ranges) - ((data.angle_max - end_angle) / data.angle_increment))

		for i in range(start_range, end_range):
			#aktuelle Distanz zum Winkel i
			d = data.ranges[i]
			if(i % 2 == 0):
				#Grenzfälle ausschließen
				if np.isnan(d):
					d = 0
				elif np.isinf(d) or d > self.R:
					d = self.R
			
				#Umrechnung in kartesische Koordinaten
				angle = start_angle + data.angle_increment * i
				if d != 0:
					self.x_vector += (1/d)*np.cos(angle)
					self.y_vector += (1/d)*np.sin(angle)

		self.x_vector = -self.x_vector  # zum Test + 10
		self.y_vector = -self.y_vector
		

		#print ("------Vektoren------")
		#print (self.x_vector)
		#print (self.y_vector)
	

	
	#lineare Geschwindigkeit brechnen
	def calculateLinearVelocity(self, x, y):
		repulsive_force = (self.x_vector ** 2 + self.y_vector ** 2) ** 0.5
		if repulsive_force == 0:
			return 0
		vel = 10/repulsive_force
		
		if abs(vel) > 1.8:
			vel = 1.8*np.sign(vel)  # eigentlich 2 testweise nur ...
		#print("------")
		#print("lin", vel)
		return vel

	# calculates the rotate velocity
	def calculateRotateVelocity(self, x, y):
		if x == 0 and y == 0:
			return 0
		angle = np.arctan2(y,x)
		if angle == 0:
			return 0
		vel = angle * 0.4
		if abs(vel) > 1.5:
			vel = 1.5 * np.sign(vel)
		#print("ang", np.rad2deg(angle))
		return vel

	def run(self):
		#alle nicht benoetigten Werte auf null setzen
		msg = Twist()
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		#waehrend Roboter an ist
		while not rospy.is_shutdown():
			#Setzen der Geschwindigkeiten und Ausgleich der Koordinatensysteme
			msg.angular.z = self.calculateRotateVelocity(self.x_vector, self.y_vector)
			msg.linear.x = self.calculateLinearVelocity(self.x_vector, self.y_vector)
			

			#msg.linear.x = 0
			#msg.angular.z = 0

			self.pub.publish(msg)
			rospy.on_shutdown(self.stop)
			self.rate.sleep()

	# stop robot when node is stopped
	def stop(self):
		Twist().linear.x = 0
		Twist().linear.y = 0
		Twist().linear.z = 0
		Twist().angular.x = 0
		Twist().angular.y = 0
		Twist().angular.z = 0

		self.pub.publish(Twist())


if __name__ == '__main__':
	try:
		Avoid().run()
	except rospy.ROSInterruptException:
		pass
