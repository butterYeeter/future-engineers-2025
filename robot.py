import ffi

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

    self.target_value = self.compass.angle()
    calib_steering(self.med_motor)

    # self.log = DataLog("RGB", "HSV", "Detected", name='../usb/log')

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