#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


achou_perigo = False
direita_perigo = False
esquerda_perigo = False

def scaneou(dado):
	global achou_perigo
	achou_perigo = False
	distancia_segura = 0.25

	for i in range(0,46,1):
		valor = dado.ranges[i]
		if valor < dado.range_max and valor > dado.range_min:
			if valor < distancia_segura:
				direita_perigo = False
				esquerda_perigo = True
				achou_perigo = True
					
	for i in range(314,359,1):
		valor = dado.ranges[i]
		if valor < dado.range_max and valor > dado.range_min:
			if valor < distancia_segura:
				direita_perigo = True
				esquerda_perigo = False
				achou_perigo = True