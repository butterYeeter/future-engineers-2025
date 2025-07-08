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

# u = UltrasonicSensor(Port.S1)
u = Ev3devSensor(Port.S1)
c = Ev3devSensor(Port.S2)

steer = Motor(Port.C)
drive = Motor(Port.B, positive_direction=Direction.CLOCKWISE)

t_ang = g.get_angle()

calib_steering(steer)

gains = [float(g) for g in argv[1:]]
print("Controller gains: ", gains)

target_distance = 200
pid1 = PIDController(target_distance, gains[0], gains[1], gains[2], 35)
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
st = watch.time()
do_thing = False
while num_turns < 12:
  left = u.read('US-DIST-CM')[0]
  da = abs(t_ang - g.get_angle())
  correction = 0


  if da > 30:
    k_ultra = 0
  else:
    k_ultra = 0.5


  if left - ld_prev > 1000 and turning == False:
  # col = c.read('NORM')[:3]
  # if is_blue(col) and turning == False:
    turning = True
    cool_down = 150
    t_ang += 90
    pid2.set_target(t_ang)

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

  if watch.time() - st > 1000 and do_thing == False:
    do_thing = True
    pid1.set_target(500)

  ld_prev = left
  correction = k_gyro * pid2.loop(g.get_angle()) + k_ultra * -pid1.loop(left)
  steer.track_target(correction)
  log.log(target_distance, left, correction)
    

  # utime.sleep_ms(200)
