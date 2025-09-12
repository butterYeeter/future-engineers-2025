#!/usr/bin/env pybricks-micropython

from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import UltrasonicSensor, Motor
from pybricks.parameters import Port
from pybricks.tools import StopWatch

from util import *
from color import ColorSensor
from gyro import Gyro
from pid import PIDController

US_WEIGHT = 0.4
GYRO_WEIGHT = 0.6

BLUE = 1
ORANGE = 2

pico_uart = UARTDevice(Port.S4, 115200, 20)
color_sensor = ColorSensor(pico_uart)
color_sensor.reset_detected()
gyro_sensor = Gyro(pico_uart)

left_us = UltrasonicSensor(Port.S1)
right_us = UltrasonicSensor(Port.S3)

gpid = PIDController(gyro_sensor.get_angle(), 2.2, 0.02, 1.5, 30, 5)
upid = PIDController(0, 1.5, -0.03, 1.2, 30, 100)

timer = StopWatch()

drive_motor = Motor(Port.B)
steer_motor = Motor(Port.D)
calib_steering(steer_motor)

wait_for_start()

us_weight = US_WEIGHT
gyro_weight = GYRO_WEIGHT
turns = 0
colors = 0
outside_us = None
outside_dir = -1
dir_change = 0
drive_motor.dc(90)
# last_angle = gyro_sensor.get_angle()
# while True:
#   # while color_sensor.get_detected() == 0:
#   color = color_sensor.get_detected()
#   if color != 0:
#     if colors == 0:
#       outside_us = right_us if color == BLUE else left_us
#       outside_dir = -1 if color == ORANGE else 1
#       dir_change = 45 if color == BLUE else -45
#     turns += 1
#     colors += 1
#     gpid.target += dir_change
#     # upid.target = 200
#     color_sensor.reset_detected()
#     print("Color Detected")
#     break

#   angle = gyro_sensor.get_angle()
#   delta_angle = abs(angle - gpid.target)

#   # if delta_angle > 30:
#   #   us_weight = 0
#   # else:
#   #   us_weight = US_WEIGHT
  
#   delta_dist = right_us.distance() - left_us.distance()
#   if abs(delta_dist) > 500:
#     us_weight = 0.7
#   else:
#     us_weight = US_WEIGHT


#   print("Delta_dist: {}".format(delta_dist))
#   correction = upid.loop(delta_dist) * us_weight + gyro_weight * gpid.loop(angle)
#   steer_motor.track_target(correction)
  
last_dist = right_us.distance() - left_us.distance()
last_time = timer.time()
delta_angle = 0
while not (turns >= 12 and delta_angle < 5):
  angle = gyro_sensor.get_angle()
  delta_angle = abs(angle-gpid.target)
  dist = right_us.distance() - left_us.distance()
  delta_dist = abs(dist)
  last_dist = dist

  if delta_angle > 30 or colors % 2 == 1:
    us_weight = 0
    gyro_weight = 1
  else:
    us_weight = US_WEIGHT
    gyro_weight = GYRO_WEIGHT

  if delta_dist > 400:
    us_weight *= 0.6

  time = timer.time()
  dt = time - last_time
  color = color_sensor.get_detected()
  if dt < 200:
    color = 0
    color_sensor.reset_detected()
  else:
    if color != 0:
      if colors == 0:
        outside_us = right_us if color == BLUE else left_us
        outside_dir = -1 if color == ORANGE else 1
        dir_change = 45 if color == BLUE else -45
      colors+=1
      gpid.target += dir_change
      last_time = time
      if colors % 2 == 0:
        turns += 1
    elif dt > 1500 and colors % 2 == 1:
      colors+=1
      gpid.target += dir_change
      last_time = time
      if colors % 2 == 0:
        turns += 1


  print("dt: {}, dl: {}, color: {}, colors: {}, turns: {}".format(dt, dist, color, colors, turns))
    
  correction = upid.loop(dist) * us_weight + gyro_weight * gpid.loop(angle)
  steer_motor.track_target(correction)


steer_motor.run_target(1000, 0)
motor_angle = drive_motor.angle() + distance_to_angle(400)
while drive_motor.angle() < motor_angle:
  angle = gyro_sensor.get_angle()
  steer_motor.track_target(gpid.loop(angle))
  
  # turns += 1