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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import encontra_objetos

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
perigo_laser = False
media1 = []
centro1 = []
area1 = 0.0

media2 = []
centro2 = []
area2 = 0.0


tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados




def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media1
	global centro1
	global area1

	global media2
	global centro2
	global area2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		perigo_laser = False
		media1, centro1, area1 = cormodule.identifica_cor1(cv_image)
		media2, centro2, area2 = cormodule.identifica_cor2(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	





## Classes - estados


class Girando(smach.State):
    def __init__(self):
    	#alinhou1 = referente ao objeto1
    	#alinhou2 = referente ao objeto2
        smach.State.__init__(self, outcomes=['perigo','alinhou1','alinhou2', 'girando'])

    def execute(self, userdata):
		global velocidade_saida

		if perigo_laser:
			return 'perigo'
		
		elif area1!=0:
			if  math.fabs(media1[0]) > math.fabs(centro1[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'girando'
			if math.fabs(media1[0]) < math.fabs(centro1[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhou1'

		elif area2!=0:
			if  math.fabs(media2[0]) > math.fabs(centro2[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'girando'
			if math.fabs(media2[0]) < math.fabs(centro2[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhou2'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'girando'

class Reage1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','perigo'])

    def execute(self, userdata):
		global velocidade_saida

		if perigo_laser:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'perigo'

		else:
			if media1 is None: #####O que significa a media ser nulo?#####
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhado'
			if  math.fabs(media1[0]) > math.fabs(centro1[0] + tolerancia_x):
				return 'alinhando'
			if math.fabs(media1[0]) < math.fabs(centro1[0] - tolerancia_x):
				return 'alinhando'
			else:
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhando'

class Reage2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','perigo'])

    def execute(self, userdata):
		global velocidade_saida

		if perigo_laser:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'perigo'

		else:
			if media2 is None: #####O que significa a media ser nulo?#####
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhado'
			if  math.fabs(media2[0]) > math.fabs(centro2[0] + tolerancia_x):
				return 'alinhando'
			if math.fabs(media2[0]) < math.fabs(centro2[0] - tolerancia_x):
				return 'alinhando'
			else:
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(1)
				return 'alinhando'

class Parar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seguro','perigo'])

    def execute(self, userdata):
		global velocidade_saida

		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		rospy.sleep(1)

		if perigo_laser:
			return 'perigo'
		else:
			return 'seguro'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor4_maq_est')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    #smach.StateMachine.add('LONGE', Longe(), 
	    #                       transitions={'ainda_longe':'ANDANDO', 
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(), 
	    #                       transitions={'ainda_longe':'LONGE'})
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou1':'REACAO1','alinhou2':'REACAO2','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO1', Reage1(),
	                            transitions={'alinhado': 'REACAO1',
	                            'alinhando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO2', Reage2(),
	                            transitions={'alinhado': 'REACAO2',
	                            'alinhando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('PERIGOSO', Parar(),
	                            transitions={'perigo': 'PERIGOSO',
	                            'seguro':'GIRANDO'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()