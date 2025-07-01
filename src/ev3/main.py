#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import DataLog, StopWatch
from robot import Robot
import queue





robot = Robot()
robot.drive_forward(speed=50)

while True:
  robot.loop()

  