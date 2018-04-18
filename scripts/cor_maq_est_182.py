#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import encontra_objetos_18
from le_scan_sonny_182 import achou_perigo, scaneou, direita_perigo, esquerda_perigo, frente_perigo, tras_perigo

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0

objeto2 = False


tolerancia_x = 20
tolerancia_y = 20
ang_speed = 0.1
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.3E9
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

delay = 0.04


frame_counter = 0

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area

	global frame_counter
	frame_counter+=1

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

		media, centro, area = encontra_objetos_18.identifica_cor(cv_image)
		if frame_counter%3 ==0:
			objeto2 = encontra_objetos_18.identifica_objeto_2(cv_image)
		
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	





## Classes - estados


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centralizou', 'girando','perigo','achou_2'])

    def execute(self, userdata):
		global velocidade_saida

		if achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'perigo'

		elif area>5000 and len(media)!=0 and len(centro)!=0:
			if math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
				return 'girando'
			if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
				return 'centralizou'

		elif encontra_objetos_18.achou_objeto:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'achou_2'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'girando'


class Reacao1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','perigo'])

    def execute(self, userdata):
		global velocidade_saida

		if achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'perigo'

		elif media is None:
			return 'alinhando'
		elif  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'alinhando'
		elif math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'
		else:
			vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'alinhado'

class Reacao2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['procurando', 'enxergando','perigo'])

    def execute(self, userdata):
		global velocidade_saida

		if achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(delay)
			return 'perigo'
		else:
			if encontra_objetos_18.achou_objeto:
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
				return 'enxergando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
				return 'procurando'


class Perigo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['perigo', 'salvo'])

    def execute(self, userdata):
		global velocidade_saida

		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		rospy.sleep(delay)

		if achou_perigo:
			if direita_perigo and esquerda_perigo and frente_perigo and tras_perigo:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
			elif esquerda_perigo or direita_perigo:
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
			elif frente_perigo:
				vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.8))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
			elif tras_perigo:
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(delay)
			return 'perigo'
		else:
			return 'salvo'

		
# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_maq_est_182')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	perigo_laser_objeto = rospy.Subscriber("/scan", LaserScan, scaneou)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'centralizou':'REAGE1','perigo':'PERIGOSO','achou_2':'REAGE2'})
	    smach.StateMachine.add('REAGE1', Reacao1(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'REAGE1','perigo':'PERIGOSO'})

	    smach.StateMachine.add('REAGE2', Reacao2(),
	                            transitions={'enxergando':'REAGE2',
	                            'procurando':'GIRANDO','perigo':'PERIGOSO'})

	    smach.StateMachine.add('PERIGOSO', Perigo(),
	                            transitions={'perigo': 'PERIGOSO',
	                            'salvo':'GIRANDO'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()