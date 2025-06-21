#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
from pybricks.nxtdevices import UltrasonicSensor
from calib import calib_steering


ev3 = EV3Brick()
big_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
med_motor = Motor(Port.C)
ultrasonic = UltrasonicSensor(Port.S3)


target_distance = 150
calib_steering(med_motor)

Kp = 0.5
Ki = 0
Kd = 0
big_motor
# big_motor.run(360)

c = med_motor.control

while True:
  error = target_distance - ultrasonic.distance()
  med_motor.run_target(540, Kp * error)
  
