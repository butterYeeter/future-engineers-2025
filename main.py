#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.nxtdevices import UltrasonicSensor
from pybricks.iodevices import Ev3devSensor
from pybricks.tools import DataLog
from calib import calib_steering
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

class Robot:
  def __init__(self):
    self.ev3 = EV3Brick()
    self.Kp = 1.5
    self.Ki = 0
    self.Kd = 0.01

    self.last_error = 0
    self.error = 0
    self.derivative = 0
    self.integral = 0

    self.actions = []
    self.current_action = None

    self.big_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
    self.med_motor = Motor(Port.C)

    self.color_sensor = Ev3devSensor(Port.S2)

    self.compass = GyroSensor(Port.S1)

    self.target_value = self.compass.angle()
    calib_steering(self.med_motor)

    self.log = DataLog("color_distance")

  def drive_forward(self, speed=75):
    self.big_motor.dc(speed)

  @staticmethod
  def color_distance(color, target=(0, 0, 255)):
    return sum((color[i]/255 - target[i]/255) ** 2 for i in range(len(color)))

  def loop(self):
    raw_angle = self.compass.angle()
    self.error = raw_angle - self.target_value
    self.integral += self.error
    self.derivative = self.error - self.last_error
    self.last_error = self.error
    self.correction = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.error
    self.med_motor.track_target(self.correction)

    if self.current_action:
      self.current_action.loop()
      if self.current_action.done:
        self.current_action = None

    rgb = self.color_sensor.read("NORM")[0:3]
    distance = self.color_distance(rgb)
    self.log.log(distance)
    if distance < 0.6:
      if not self.current_action:
        print("Found blue! Turning")
        self.current_action = Turn(self, Direction.COUNTERCLOCKWISE)

robot = Robot()
robot.drive_forward(speed=50)

while True:
  robot.loop()

  