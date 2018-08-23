import socket
import cv2
import numpy as np
import threading
import signal
import sys
from socket import timeout
import requests
from io import open as iopen


# import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage



def receive_image(UDP_PORT,timeout_limit,name,msg):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT)) 
	sock.settimeout(timeout_limit)
	sockets_list.append(sock)
	bridge = CvBridge()
	while True:
		try:
			data = sock.recv(65536) 
			if name == 'left' or name =='right':
				image_data = CompressedImage()
				image_data.deserialize(data)
				image = bridge.compressed_imgmsg_to_cv2(image_data)
				# np_arr = np.fromstring(data, np.uint8)
				# image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
			cv2.imwrite('{}.png'.format(name),image)
			print('{} saved'.format(msg))
		except timeout:
			print('Error: {} Timeout'.format(msg))
			sys.exit(1)


def receive_laser(UDP_PORT,timeout_limit):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT)) 
	sock.settimeout(timeout_limit)
	sockets_list.append(sock)
	while True:
		try:
			data = sock.recv(65536)
			laser_data = LaserScan()
			laser_data.deserialize(data)
			np.save('laser',laser_data)
			# print('Laser Data Received\n')
		except timeout:
			print('Error: Laser Data Timeout')
			sys.exit(1)

def receive_rededge(UDP_PORT,timeout_limit):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT)) 
	sock.settimeout(timeout_limit)
	sockets_list.append(sock)
	while True:
		try:
			data = sock.recv(65536)
			file_names = data.split(";")
			for name in file_names:
				r = requests.get(name)
				with iopen(name[-5:], 'wb') as file:
					file.write(r.content)
		except timeout:
			print('Error: Rededge Timeout')
			sys.exit(1)

def handler(signum, frame):
    for s in sockets_list:
    	s.close()
    # for t in thread_list:
    # 	t.close()
    sys.exit(0)

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = socket.gethostbyname(socket.gethostname())
    finally:
        s.close()
    return IP



if __name__ == '__main__':

	# get UDP parameters
	UDP_IP = get_ip()
	timeout_limit = 20.0
	port_list = [5005,5006]
	name_list = ['left','right']
	msg_list = ['Left Image','Right Image']
	laserscan_port = 5007
	rededge_port = 5008

	print('Timeout setting: {}s'.format(timeout_limit))
	print('IP Address is: {}'.format(UDP_IP))
	for cnt in range(len(port_list)):
		print('{} Port: {}'.format(msg_list[cnt],port_list[cnt]))
	print('Laser Scan Port: {}'.format(laserscan_port))
	print('Rededge Port: {}'.format(rededge_port))

	#start thread for receving
	sockets_list = []
	thread_list = []
	signal.signal(signal.SIGINT, handler)
	try:
		for cnt in range(len(port_list)):
			thread_list.append(
				threading.Thread(target=receive_image, 
					args=(port_list[cnt],timeout_limit,name_list[cnt],msg_list[cnt])
					)
				)
		thread_list.append(
			threading.Thread(target=receive_laser,
				args=(laserscan_port,timeout_limit)
				)
			)
		thread_list.append(
			threading.Thread(target=receive_rededge,
				args=(rededge_port,timeout_limit)
				)
			)
		for t in thread_list:
			t.start()
		for t in thread_list:
			t.join()
	finally:
		for s in sockets_list:
			s.close()