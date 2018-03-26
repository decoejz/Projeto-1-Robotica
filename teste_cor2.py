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

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def identifica_objeto_1(frame):
	pass
	#Essa função deve dizer se o objeto 1 foi identificado ou não.
	#Ela retorna dois valores:
	#1) True or False caso tenha sido identificado
	#2) A distancia do robo até o objeto.
def identifica_objeto_2(frame):
	pass
	#Essa função deve dizer se o objeto 2 foi identificado ou não.
	#Ela retorna um valor:
	#True or False caso tenha sido identificado

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	


if __name__=="__main__":

	rospy.init_node("cor")
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	#recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:
		vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

		while not rospy.is_shutdown():
			obj1, distancia = identifica_objeto_1(frame)
			obj2 = identifica_objeto_2(frame)
			if obj1:
				if distancia>20:
					vel = Twist(Vector3(1,0,0), Vector3(0,0,0))
				elif distancia>10:
					vel = Twist(Vector3(0.7,0,0), Vector3(0,0,0))
				elif distancia>5:
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
				elif distancia>2:
					vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
				else:
					vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			elif obj2:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,1)
				velocidade_saida.publish(vel)


			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")






