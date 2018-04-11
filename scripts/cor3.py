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
from findobjs import findobj1


bridge = CvBridge()

cv_image = None

perigo = False
objeto_1 = False
objeto_2 = False

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais





def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global perigo
	global objeto_1
	global objeto_2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		perigo = False
		objeto_1 = findobj1(cv_image)
		objeto_2 = False
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	


if __name__=="__main__":

	rospy.init_node("cor3")

	# Para usar a Raspberry Pi
	topico_raspberry_camera = "/raspicam_node/image/compressed"
	# Para usar a webcam 
	topico_webcam = "/cv_camera/image_raw/compressed"

	topico_imagem = topico_raspberry_camera

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if perigo:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			elif objeto_1:
				vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
			elif objeto_2:
				vel = Twist(Vector3(0,0,0), Vector3(np.pi/8,0,0))
				velocidade_saida.publish(vel)
				rospy.sleep(2)
				vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")