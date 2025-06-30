#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.nxtdevices import UltrasonicSensor
from pybricks.iodevices import Ev3devSensor
from pybricks.tools import DataLog
from calib import calib_steering
import queue

def rgb_to_hsv(color):
    r, g, b = [x/255.0 for x in color]

    max_val = max(r, g, b)
    min_val = min(r, g, b)
    diff = max_val - min_val
    
    v = max_val * 100
    
    s = 0.0 if max_val == 0 else (diff / max_val) * 100
    
    if diff == 0:
        h = 0
    else:
        if max_val == r:
            h = 60 * (((g - b) / diff) % 6)
        elif max_val == g:
            h = 60 * (((b - r) / diff) + 2)
        else:  # max_val == b
            h = 60 * (((r - g) / diff) + 4)
    
    return (round(h, 2), round(s, 2), round(v, 2))

def is_blue(rgb_tuple, threshold=None):
    # Default blue threshold (H:200-260°, S:40-100%, V:40-100%)
    default_threshold = (200, 260, 40, 100, 40, 100)
    h_min, h_max, s_min, s_max, v_min, v_max = threshold or default_threshold
    
    h, s, v = rgb_to_hsv(rgb_tuple)
    
    # Check if within all thresholds
    return (h_min <= h <= h_max and 
            s_min <= s <= s_max and 
            v_min <= v <= v_max), (h, s, v)

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
    self.cooldown = 100

  def loop(self):
    if self.direction == Direction.COUNTERCLOCKWISE and not self.has_turned:
      self.robot.target_value -= 90
      self.has_turned = True
    if self.direction == Direction.COUNTERCLOCKWISE and not self.has_turned:
      self.robot.traget_value += 90
      self.has_turned = True

    self.cooldown -= 1
    if self.cooldown <= 0:
      self.done = True

class PIDController:
  def __init__(self, robot):
    self.robot = robot

  def loop(self):
    pass



robot = Robot()
robot.drive_forward(speed=20)
usbser_init('./serial_ev3/libusbser.so')
ret = usbinit('/dev/ttyACM0', 115200)

while True:
  robot.loop()

  