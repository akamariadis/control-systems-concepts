class PIDController:
    """Discrete PID controller with anti-windup clamping."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limits: tuple[float, float] = (-float("inf"), float("inf")),
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.lower_limit, self.upper_limit = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def reset(self) -> None:
        """Reset internal accumulator and derivative history."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def update(self, setpoint: float, measured_value: float, dt: float) -> float:
        """Compute the control signal for the current sampling interval."""
        if dt <= 0.0:
            return 0.0

        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term accumulation
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative term
        if self._first_run:
            d_term = 0.0
            self._first_run = False
        else:
            d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        # Compute raw output and apply actuator limits
        raw_output = p_term + i_term + d_term
        clamped_output = max(self.lower_limit, min(raw_output, self.upper_limit))

        # Anti-windup: revert integration if saturated in the direction of error
        if raw_output != clamped_output:
            if (raw_output > self.upper_limit and error > 0) or (
                raw_output < self.lower_limit and error < 0
            ):
                self._integral -= error * dt

        return clamped_output
