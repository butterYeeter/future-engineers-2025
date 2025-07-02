class PIDController:
    def __init__(self, target, Kp=1.0, Ki=0.0, Kd=0.0, output_limit=40, integral_limit=100):
        self.target = target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0
        self.last_error = 0

    def loop(self, current_value):
        error = self.target - current_value
        derivative = error - self.last_error
        self.last_error = error
        new_integral = self.integral + error

        correction_unclamped = (
            self.Kp * error +
            self.Ki * new_integral +
            self.Kd * derivative
        )

        correction = max(min(self.output_limit, correction_unclamped), -self.output_limit)

        if correction == correction_unclamped:
            self.integral = new_integral

            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        return correction

    def set_target(self, value):
        self.target = value
