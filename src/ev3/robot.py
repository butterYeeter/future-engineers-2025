from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, DataLog
from pid import PIDController
from gyro import Gyro
from sys import argv

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

    # Use PID values from commandline if supplied
    if len(argv) < 4:
      self.pid = PIDController(self.target_value, 2.2, 0.02, 1.5)
    else:
      self.pid = PIDController(self.target_value, float(argv[1]), float(argv[2]), float(argv[3]), 32)

    wait(500)
    self.zero_steering()

    self.log = DataLog("TA", "A", "C", "LD", "dLD", name='../logs/log')

  def drive_forward(self, speed=75):
    self.big_motor.dc(speed)
  
  def zero_steering(self):
    m = self.med_motor
    angle_right = m.run_until_stalled(720, duty_limit=76)
    angle_left = m.run_until_stalled(-720, duty_limit=76)
    m.run_target(90, (angle_left + angle_right)/2)
    m.reset_angle(0)

  def loop(self):
    self.pid.set_target(self.target_value)
    current_angle = self.gyro.get_angle()
    correction = self.pid.loop(current_angle)
    self.med_motor.track_target(correction)


    # Turn if delta distance > 1500 and the current action is None
    left_distance = self.left_ultrasonic.distance()
    left_delta = left_distance - self.left_distance_prev
    if left_delta > 1500 and self.current_action == None:
      self.current_action = Turn(self, Direction.COUNTERCLOCKWISE)
    self.left_distance_prev = left_distance


    # Call the current actions loop function
    if self.current_action:
      self.current_action.loop()
      if self.current_action.done:
        self.current_action = None

    
    self.log.log(self.target_value, current_angle, correction, left_distance, left_delta)