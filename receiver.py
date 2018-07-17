import socket
import cv2
import numpy as np
import threading
import signal
import sys
from socket import timeout


# import rospy
from sensor_msgs.msg import LaserScan



def receive_left(UDP_PORT,timeout_limit):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT)) 
	sock.settimeout(timeout_limit)
	sockets_list.append(sock)
	while True:
		try:
			data = sock.recv(65536) 
			np_arr = np.fromstring(data, np.uint8)
			image_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
			cv2.imwrite('left.png',image_np)
			print('Left Image saved')
		except timeout:
			print('Error: Left Image Timeout')
			sys.exit(1)

def receive_right(UDP_PORT,timeout_limit):
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT)) 
	sock.settimeout(timeout_limit)
	sockets_list.append(sock)
	while True:
		try:
			data = sock.recv(65536) 
			np_arr = np.fromstring(data, np.uint8)
			image_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
			cv2.imwrite('right.png',image_np)
			print('Right Image saved')
		except timeout:
			print('Error: Right Image Timeout')
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
			print('Laser Data Received\n')
		except timeout:
			print('Error: Laser Data Timeout')
			sys.exit(1)

def handler(signum, frame):
    for s in sockets_list:
    	s.close()
    for t in thread_list:
    	t.close()
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
	timeout_limit = 10.0
	leftimage_port = 5005
	rightimage_port = 5006
	laserscan_port = 5007

	print('Timeout setting: {}s'.format(timeout_limit))
	print('IP Address is: {}'.format(UDP_IP))
	print('Left Image Port: {}'.format(leftimage_port))
	print('Right Image Port: {}'.format(rightimage_port))
	print('Laser Scan Port: {}'.format(laserscan_port))

	#start thread for receving
	sockets_list = []
	thread_list = []
	signal.signal(signal.SIGINT, handler)
	try:
   		thread_list.append(threading.Thread(target=receive_left, args=(leftimage_port,timeout_limit)))
   		thread_list.append(threading.Thread(target=receive_right, args=(rightimage_port,timeout_limit)))
   		thread_list.append(threading.Thread(target=receive_laser,args=(laserscan_port,timeout_limit)))
   		for t in thread_list:
   			t.start()
   		for t in thread_list:
   			t.join()
	except:
   		pass
   	finally:
   		for s in sockets_list:
   			s.close()


