#!/usr/bin/env python3

#scp pi@10.42.0.2:/home/pi/catkin_ws/src/orientation/src/send_move_lidar.py /home/oskar/Desktop/send_move_lidar.py
#scp scp /home/oskar/Desktop/scripte_bachelor/stop_move.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/


import rospy
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
import serial

ser = serial.Serial(
		'/dev/ttyS0', 
		baudrate = 115200,
		parity = serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize =serial.EIGHTBITS,
		timeout = 1
		)

ser.close()
ser.open()
def listener():
	global ser
	array = bytes(f'<{0},{0},{0},{1}>\n', 'utf-8')
	ser.write(array)


if __name__ == "__main__":
	rospy.init_node("move_punlisher")
	listener()
