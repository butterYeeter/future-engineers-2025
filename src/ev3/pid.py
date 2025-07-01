class PIDController:
  def __init__(self, target_value, Ki=1, Kp=0, Kd=0):
    self.target_value = target_value
    self.integral = 0
    self.derivative = 0
    self.last_error = 0
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

  def loop(self, current_value):
    error = self.target_value - current_value
    self.integral += error
    self.derivative = error - self.last_error
    self.last_error = error
    
    correction = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
    return (correction, error)