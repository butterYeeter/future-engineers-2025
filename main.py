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


target_value = 200
Kp = 10
Ki = 0
Kd = 0.5

last_error = 0
error = 0
derivative = 0
integral = 0

big_motor.dc(100)

while True:
  error = target_value - ultson.distance()
  integral += error
  derivative = error - last_error
  last_error = error
  correction = Kp * error + Ki * integral + Kd * error
  # med_motor.run_until_stalled(correction, duty_limit=30)
  med_motor.run(correction)
  print(correction, med_motor.angle())
  buttons = ev3.buttons.pressed()
  if Button.UP in buttons:
    med_motor.run_target(360, 0)
    break