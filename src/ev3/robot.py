from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, DataLog
from calib import calib_steering
from pid import PIDController
from gyro import Gyro

class Robot:
  def __init__(self):
    self.ev3 = EV3Brick()

    self.actions = []
    self.current_action = None

    self.big_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
    self.med_motor = Motor(Port.C)

    self.gyro = Gyro()
    self.gyro.init()
    self.target_value = self.gyro.get_angle()
    # self.pid = PIDController(self.target_value, 0.005, 0.0002, 0.05)
    self.integral = 0
    self.derivative = 0
    self.last_error = 0
    wait(1000)
    calib_steering(self.med_motor)


    self.log = DataLog("Target angle", "Current Angle", "PID Correction", "Error", name='../logs/log')

  def drive_forward(self, speed=75):
    self.big_motor.dc(speed)

  @staticmethod
  def color_distance(color, target=(0, 0, 255)):
    return sum((color[i]/255 - target[i]/255) ** 2 for i in range(len(color)))

  def loop(self):
    raw_angle = self.gyro.get_angle()
    # print(raw_angle)
    correction, err = self.pid.loop(raw_angle)
    correction = max(min(40, correction), -40)
    self.log.log(self.target_value, raw_angle, correction, err)
    self.med_motor.track_target(correction)

    if self.current_action:
      self.current_action.loop()
      if self.current_action.done:
        self.current_action = None