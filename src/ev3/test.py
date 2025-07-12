#!/usr/bin/env pybricks-micropython
from gyro import Gyro
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import UltrasonicSensor, Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import DataLog, StopWatch, wait
import utime
from math import cos, radians
from pid import PIDController
from sys import argv
from calib import calib_steering
from util import is_blue

g = Gyro()

g.init()

u = Ev3devSensor(Port.S1)
c = Ev3devSensor(Port.S2)

steer = Motor(Port.C)
drive = Motor(Port.B, positive_direction=Direction.CLOCKWISE)

t_ang = g.get_angle()

calib_steering(steer)

target_distance = 200
pid1 = PIDController(target_distance, 0.8, 0.01, 0.7, 35)
pid2 = PIDController(t_ang, 2.2, 0.02, 1.5, 35)

log = DataLog("target len", "len", "correciton", name="../logs/log", timestamp=False)

watch = StopWatch()

current_action = None
turning = False
cool_down = 250
ld_prev = u.read('US-DIST-CM')[0]*cos(radians(abs(t_ang - g.get_angle())))

k_gyro = 0.5
k_ultra = 0.5

ev = EV3Brick()
start = False
while start == False:
  buttons = ev.buttons.pressed()
  if Button.CENTER in buttons:
    start = True

  wait(10)

wait(2000)
drive.dc(100)
num_turns = 0

while num_turns < 12:
  left = u.read('US-DIST-CM')[0]
  da = abs(t_ang - g.get_angle())
  correction = 0

  if da > 30:
    k_ultra = 0
  else:
    k_ultra = 0.5

  # if left - ld_prev > 1000 and turning == False:
  #   turning = True
  #   cool_down = 150
  #   t_ang += 90
  #   pid2.set_target(t_ang)

  if turning == True:
    cool_down -= 1
    k_ultra = 0
    if cool_down == 0:
      turning = False
      k_ultra = 0.5
      num_turns += 1
  else:
    if left - ld_prev > 500:
      k_ultra *= 0.8

  ld_prev = left
  correction = k_gyro * pid2.loop(g.get_angle()) + k_ultra * -pid1.loop(left)
  steer.track_target(correction)
  log.log(target_distance, left, correction)