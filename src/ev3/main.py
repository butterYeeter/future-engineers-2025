#!/usr/bin/env pybricks-micropython
from pybricks.parameters import Button
from pybricks.tools import wait
from robot import Robot
import sys


print(sys.argv)


robot = Robot()

start = False

while start == False:
  buttons = robot.ev3.buttons.pressed()
  if Button.CENTER in buttons:
    start = True

  wait(10)

robot.drive_forward(speed=50)

while True:
  robot.loop()

  