#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.nxtdevices import UltrasonicSensor
from pybricks.iodevices import Ev3devSensor
from calib import calib_steering


ev3 = EV3Brick()
big_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
med_motor = Motor(Port.C)
calib_steering(med_motor)

compass = Ev3devSensor(Port.S1)
ultson = UltrasonicSensor(Port.S2)


target_value = 165
Kp = 1.5
Ki = 0
Kd = 0.01

last_error = 0
error = 0
derivative = 0
integral = 0

big_motor.dc(50)

while True:
  error = compass.read('COMPASS')[0] - target_value
  integral += error
  derivative = error - last_error
  last_error = error
  correction = Kp * error + Ki * integral + Kd * error
  med_motor.track_target(correction)

  buttons = ev3.buttons.pressed()
  if Button.LEFT in buttons:
    target_value = 75
  elif Button.RIGHT in buttons:
    target_value = 255
  elif Button.UP in buttons:
    target_value = 165