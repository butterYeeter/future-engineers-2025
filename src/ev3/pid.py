class PIDController:
  def __init__(self, target, Kp=1, Ki=0, Kd=0):
    self.target = target
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

    self.integral = 0
    self.derivative = 0
    self.last_error = 0

  def loop(self, current_value):
    error = self.target - current_value
    self.integral += error
    self.derivative = error - self.last_error
    self.last_error = error

    correction = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative

    correction = max(min(40, correction), -40)

    return (correction, error)

  def set_target(self, value):
    self.target = value

