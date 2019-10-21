import RPi.GPIO as GPIO
import numpy as np
from dronekit import connect, LocationGlobalRelative
import time
from math import *

# Connect to the Vehicle
print("connecting to groung station")
vehicle1 = connect("127.0.0.1:14551")     #Ground Station
print("Connecting to vehicle on 127.0.0.1:14550")
vehicle2 = connect("127.0.0.1:14550")      #Drone

out1 = 31
dir1 = 33
out2 = 35
dir2 = 37

steps_per_rotation1 = 400
steps_per_rotation2 = 400
steps_per_degree1 = steps_per_rotation1/360.0
steps_per_degree2 = steps_per_rotation2/360.0
pulse_freq = 100.0
time_period = 1/pulse_freq
time_delay = 0.8

#Current Position of ATS
#curr_angle_steps = 0
#curr_Pitch_angle_steps = 0

def move():
  alt1 = vehicle1.location.global_frame.alt
  lat1 = vehicle1.location.global_frame.lat
  lon1 = vehicle1.location.global_frame.lon
  alt2 = vehicle2.location.global_frame.alt
  lat2 = vehicle2.location.global_frame.lat
  lon2 = vehicle2.location.global_frame.lon
  Altitude = alt1-alt2
  
  
  lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
  

  dlon = lon2 - lon1
  dlat = lat2 - lat1
  a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
  c = 2 * atan2(sqrt(a), sqrt(1-a))
  Base = 6371 * c
  
  head2= vehicle1.heading
  dLon = lon2 - lon1;
  y = sin(dLon) * cos(lat2);
  x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  bearing = np.rad2deg(atan2(y, x));
  #print("b0",bearing)
  if bearing < 0: bearing+= 360
  if (abs(bearing-head2)<=180):
    angle = bearing-head2
  elif bearing-head2>180:
    angle = -(360-(bearing-head2))
  elif bearing - head2<-180:
    angle = 360+bearing-head2


  ground_pitch= vehicle2.attitude.pitch
  Pitch_angle = atan2(Altitude,Base*1000)
  pitch_angle = degrees(Pitch_angle - ground_pitch)

  angle_steps = int(steps_per_degree1 * angle)
  Pitch_angle_steps = int(steps_per_degree2 * pitch_angle)
  
   #To change direction and make step value always positive

  if angle_steps < 0 :
      angle_steps *= -1
      GPIO.output(dir1, GPIO.HIGH)
  else:
      GPIO.output(dir1, GPIO.LOW)
  if Pitch_angle_steps < 0 :
      Pitch_angle_steps *= -1
      GPIO.output(dir2, GPIO.HIGH)
  else:
      GPIO.output(dir2, GPIO.LOW)
  
  print("Heading: ", head2)
  print(" bearing: ", bearing)
  '''
  print("Lat1: ", degrees(lat1))
  print("Lon1: ", degrees(lon1))
  print("Alt1: ", alt1)
  print("Lat2: ", degrees(lat2))
  print("Lon2: ", degrees(lon2))
  print("Alt2: ", alt2)'''
  return angle, pitch_angle, angle_steps, Pitch_angle_steps


GPIO.setmode(GPIO.BOARD)
GPIO.setup(out1,GPIO.OUT)
GPIO.setup(dir1,GPIO.OUT)
GPIO.setup(out2,GPIO.OUT)
GPIO.setup(dir2,GPIO.OUT)




def turn(deg1_step, deg2_step):
	for i in range(deg1_step):
		GPIO.output(out1, GPIO.HIGH)
		time.sleep(time_period/2.0)
		GPIO.output(out1, GPIO.LOW)
		time.sleep(time_period/2.0)
	for i in range(deg2_step):
		GPIO.output(out1, GPIO.HIGH)
		time.sleep(time_period/2.0)
		GPIO.output(out1, GPIO.LOW)
		time.sleep(time_period/2.0)
   

while True:
	try:			
		 a,b,c,d = move()
		 turn(c, d)
		 print(" bearing_angle,pitch_angle,bearing_steps,pitch_steps")
		 print(a,b,c,d)
		 #time.sleep(0.5)
	except (KeyboardInterrupt, SystemExit):
		GPIO.cleanup()
		print("Exit")




