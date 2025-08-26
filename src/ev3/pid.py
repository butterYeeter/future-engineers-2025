# PID Controller Class

class PIDController:
    def __init__(self, target, Kp=1.0, Ki=0.0, Kd=0.0, output_limit=40, integral_limit=100):
        # Controller state
        self.target = target    # Value to maintain
        self.integral = 0
        self.last_error = 0
        
        # Controller gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Limit the controller output range and integral limit
        self.output_limit = output_limit
        self.integral_limit = integral_limit

    # Function that gets called every loop interation to update our controller state,
    # and output a correction
    @micropython.native
    def loop(self, current_value):
        # Calculate new state(error, integral, derivative)
        error = self.target - current_value
        derivative = error - self.last_error
        self.last_error = error
        new_integral = self.integral + error

        # Our unclamped correction
        correction_unclamped = (
            self.Kp * error +
            self.Ki * new_integral +
            self.Kd * derivative
        )

        # Clamp the correction within limits
        correction = max(min(self.output_limit, correction_unclamped), -self.output_limit)

        # If our clamped correction equals the unclamped(i.e. we are within limits), 
        # update our integral state
        if correction == correction_unclamped:
            self.integral = new_integral

            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        return correction

    def set_target(self, value):
        self.target = value
