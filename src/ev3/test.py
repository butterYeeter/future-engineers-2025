#!/usr/bin/env pybricks-micropython
from gyro import Gyro
from pybricks.ev3devices import UltrasonicSensor, Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import DataLog, StopWatch
import utime
from math import cos, radians
from pid import PIDController
from sys import argv
from calib import calib_steering


class Action:
  def __init__(self):
    self.done = False

  def loop(self):
    pass

class Turn(Action):
  def __init__(self, direction):
    super().__init__()
    self.direction = direction
    self.has_turned = False
    self.cooldown = 250

    if direction == Direction.COUNTERCLOCKWISE:
      self.robot.target_value += 90
    if direction == Direction.CLOCKWISE:
      self.robot.target_value -= 90

  def loop(self):
    self.cooldown -= 1
    if self.cooldown <= 0:
      self.done = True


g = Gyro()

g.init()

u = UltrasonicSensor(Port.S1)

steer = Motor(Port.C)
drive = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)

t_ang = g.get_angle()

calib_steering(steer)

gains = [float(g) for g in argv[1:]]
print("Controller gains: ", gains)

target_distance = 150
pid1 = PIDController(target_distance, gains[0], gains[1], gains[2])
pid2 = PIDController(t_ang, 2.2, 0.02, 1.5)


log = DataLog("target len", "len", "correciton", name="../logs/log", timestamp=False)

watch = StopWatch()

current_action = None
turning = False
cool_down = 250
ld_prev = u.distance()*cos(radians(abs(t_ang - g.get_angle())))

drive.dc(80)
while True:
  left = u.distance()
  da = abs(t_ang - g.get_angle())
  correction = 0


  if left - ld_prev > 1000 and turning == False:
    # turning = True
    cool_down = 100
    t_ang += 90
    pid2.set_target(t_ang)

  if turning == True:
    correction = pid2.loop(g.get_angle())
    cool_down -= 1
    if cool_down == 0:
      turning = False
  else:
    correction = -pid1.loop(left)

  steer.track_target(correction)
  log.log(target_distance, left, correction)
    

  # utime.sleep_ms(200)
