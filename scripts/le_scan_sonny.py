#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


achou_perigo = False

def scaneou(dado):
	global achou_perigo
	achou_perigo = False
	distancia_segura = 0.25
	for i in range(0,46,1):
		valor = dado.ranges[i]
		if valor < dado.range_max and valor > dado.range_min:
			if valor < distancia_segura:
				if i <=0:
					achou_perigo = True
				else:
					achou_perigo = True
	for i in range(314,359,1):
		valor = dado.ranges[i]
		if valor < dado.range_max and valor > dado.range_min:
			if valor < distancia_segura:
				if i <=0:
					achou_perigo = True
				else:
					achou_perigo = True

	

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	while not rospy.is_shutdown():
		print(achou_perigo)
		rospy.sleep(2)