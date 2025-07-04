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
  def __init__(self, robot):
    self.robot = robot
    self.done = False

  def loop(self):
    pass

class Turn(Action):
  def __init__(self, robot, direction):
    super().__init__(robot)
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

initial = g.get_angle()

calib_steering(steer)

gains = [float(g) for g in argv[1:]]
print("Controller gains: ", gains)

target_distance = 300
pid1 = PIDController(target_distance, gains[0], gains[1], gains[2])
pid2 = PIDController(initial, 2.2, 0.02, 1.5)


log = DataLog("target len", "len", "correciton", name="../logs/log", timestamp=False)

watch = StopWatch()

drive.dc(80)
while True:
  left = u.distance()
  da = abs(initial - g.get_angle())
  correction = pid1.loop(left*cos(radians(da)))
  steer.track_target(-correction)
  log.log(target_distance, left, -correction)

  # utime.sleep_ms(200)
