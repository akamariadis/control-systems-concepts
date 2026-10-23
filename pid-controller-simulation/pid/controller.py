class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limits: tuple[float, float] = (-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lower_limit, self.upper_limit = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def update(self, setpoint: float, measured_value: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative term
        if self._first_run:
            d_term = 0.0
            self._first_run = False
        else:
            d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        # Compute raw output and clamp to actuator limits
        output = p_term + i_term + d_term
        clamped_output = max(self.lower_limit, min(output, self.upper_limit))

        # Anti-windup: clamp integral if output is saturated
        if output != clamped_output and (output * error > 0):
            self._integral -= error * dt

        return clamped_output
