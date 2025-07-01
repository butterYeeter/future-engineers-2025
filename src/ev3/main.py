#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import DataLog, StopWatch
from robot import Robot
import queue


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
    self.cooldown = 400

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




    # color sensor stuff...
    # rgb = self.color_sensor.read('NORM')[0:3]
    # bl, hsv = is_blue(rgb)
    # self.log.log(rgb, hsv, bl)
    # if bl:
    #   if not self.current_action:
    #     self.current_action = Turn(self, Direction.COUNTERCLOCKWISE)

robot = Robot()
robot.drive_forward(speed=60)

while True:
  robot.loop()

  