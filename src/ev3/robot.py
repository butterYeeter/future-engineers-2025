from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, DataLog
from pid import PIDController
from gyro import Gyro

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


class Robot:
  def __init__(self):
    self.ev3 = EV3Brick()

    self.actions = []
    self.current_action = None

    self.big_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
    self.med_motor = Motor(Port.C)

    self.gyro = Gyro()
    self.gyro.init()

    self.left_ultrasonic = UltrasonicSensor(Port.S1)
    self.left_distance_prev = self.left_ultrasonic.distance()

    self.target_value = self.gyro.get_angle()
    self.pid = PIDController(self.target_value, 0.8, 0.001, 0.01)

    wait(500)
    self.zero_steering()

    self.log = DataLog("TA", "A", "C", "E", name='../logs/log')

  def drive_forward(self, speed=75):
    self.big_motor.dc(speed)
  
  def zero_steering(self):
    m = self.MedMotor
    angle_right = m.run_until_stalled(540, duty_limit=70)
    angle_left = m.run_until_stalled(-540, duty_limit=70)
    m.run_target(90, (angle_left + angle_right)/2)
    m.reset_angle(0)

  def loop(self):
    current_angle = self.gyro.get_angle()
    correction, err = self.pid.loop(current_angle)
    self.med_motor.track_target(correction)

    self.log.log(self.target_value, current_angle, correction, err)

    left_distance = self.left_ultrasonic.distance()
    if left_distance - self.left_distance_prev > 1500:
      self.current_action = Turn()

    if self.current_action:
      self.current_action.loop()
      if self.current_action.done:
        self.current_action = None