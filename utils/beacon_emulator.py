# coding: utf-8
from beacon_messages import messages
import time
import serial


clockwise_direction = True
current_angle = 0
angle_speed = 1
alg_type = 0

# 0 - вращение с постоянной скоростью
if alg_type == 0:
	with serial.Serial('/dev/ttyUSB0', 9600, timeout=1) as ser:
		while True:
			if clockwise_direction:
				msg = messages[current_angle]
				current_angle += angle_speed
			else:
				msg = messages[0 - current_angle]
				current_angle -= angle_speed

			while ser.in_waiting:
				print(ser.readline())
			ser.write(msg)
			time.sleep(1)

			if current_angle >= 360:
				current_angle = 0
