# coding utf-8
from math import floor


default_azimuth = 10
beacon_azimuth = 315

default_angle = 90
current_angle = 90


azimuth_delta = 0

d = beacon_azimuth - default_azimuth
delta = abs(d) % 360
# if delta > 180:
# 	delta = 360 - delta

# if not ((d >= 0 and d <= 180) or (d <= -180 and d >= -360)):
# 	delta *= -1

print(delta) 


def modulo(a, b):
	return a - floor(a / b) * b


delta = modulo((d ), 360) 
print(delta)
